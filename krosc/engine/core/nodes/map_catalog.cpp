#include "krosc/nodes/map_catalog.hpp"

#include "krosc/msgs/nav2_srv.hpp"

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <sstream>
#include <string>
#include <utility>

#include "krosc/qos.hpp"

namespace krosc::nodes {

namespace {

std::string timestamp_name() {
  const std::time_t now = std::time(nullptr);
  std::tm local{};
#if defined(_WIN32)
  localtime_s(&local, &now);
#else
  localtime_r(&now, &local);
#endif
  char buffer[64] = {};
  std::strftime(buffer, sizeof(buffer), "map_%Y%m%d_%H%M%S", &local);
  return buffer;
}

}  // namespace

MapCatalog::MapCatalog(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)) {
  const auto positive = [](const double& value) { return value > 0.0; };
  maps_directory_ = declare_parameter<std::string>("maps_directory", std::string(""));
  map_frame_ = declare_parameter<std::string>("map_frame", std::string("map"));
  list_period_ = declare_parameter<double>("list_period", 1.0, positive);
  map_server_ = declare_parameter<std::string>("map_server", std::string("map_server"));
  map_saver_ = declare_parameter<std::string>("map_saver", std::string("map_saver"));
  map_change_topic_ = declare_parameter<std::string>("map_change_topic", std::string("map_change"));
  map_save_topic_ = declare_parameter<std::string>("map_save_topic", std::string("map_save"));
  map_delete_topic_ =
      declare_parameter<std::string>("map_delete_topic", std::string("map_delete"));
  map_unload_topic_ = declare_parameter<std::string>("map_unload_topic", std::string("map_unload"));
  map_list_topic_ = declare_parameter<std::string>("map_list_topic", std::string("map_list"));
  map_status_topic_ = declare_parameter<std::string>("map_status_topic", std::string("map_status"));
  slam_mode_topic_ = declare_parameter<std::string>("slam_mode_topic", std::string("slam_mode"));

  subscribe<msgs::String>(
      map_change_topic_.value(), [this](const msgs::String& message) { on_map_change(message); });
  subscribe<msgs::String>(
      map_save_topic_.value(), [this](const msgs::String& message) { on_map_save(message); });
  subscribe<msgs::String>(
      map_delete_topic_.value(), [this](const msgs::String& message) { on_map_delete(message); });
  subscribe<msgs::Empty>(
      map_unload_topic_.value(), [this](const msgs::Empty& message) { on_map_unload(message); });
  create_timer(list_period_.value(), [this] { refresh(); });
  refresh();
  publish_status("ready");
  logger().info("MapCatalog: maps=%s", directory().string().c_str());
}

std::filesystem::path MapCatalog::directory() const {
  const std::string value = maps_directory_.value();
  return value.empty() ? std::filesystem::current_path() : std::filesystem::path(value);
}

std::filesystem::path MapCatalog::map_path(const std::string& name) const {
  return directory() / name / "map";
}

bool MapCatalog::safe_name(const std::string& name) {
  return !name.empty() && name != "." && name != ".." && name.find('/') == std::string::npos &&
         name.find('\\') == std::string::npos;
}

std::string MapCatalog::loaded_name() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return loaded_name_;
}

std::vector<std::string> MapCatalog::names() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return names_;
}

std::string MapCatalog::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

void MapCatalog::refresh() {
  std::vector<std::string> found;
  std::error_code error;
  const auto base = directory();
  if (std::filesystem::exists(base, error)) {
    for (const auto& entry : std::filesystem::directory_iterator(base, error)) {
      if (error) {
        break;
      }
      if (!entry.is_directory(error)) {
        continue;
      }
      std::error_code exists_error;
      if (std::filesystem::exists(entry.path() / "map.yaml", exists_error)) {
        found.push_back(entry.path().filename().string());
      }
    }
  }
  std::sort(found.begin(), found.end());
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (found != names_) {
      names_ = found;
      changed = true;
    }
  }
  if (changed) {
    publish_list();
  }
}

void MapCatalog::publish_list() {
  std::vector<std::string> names;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    names = names_;
  }
  std::ostringstream text;
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index != 0) {
      text << '\n';
    }
    text << names[index];
  }
  publish(map_list_topic_.value(), msgs::String{text.str()}, QOS_LATCHED);
}

void MapCatalog::publish_status(const std::string& text) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = text;
  }
  publish(map_status_topic_.value(), msgs::String{text}, QOS_LATCHED);
}

void MapCatalog::publish_slam_mode(const std::string& mode) {
  publish(slam_mode_topic_.value(), msgs::String{mode}, QOS_LATCHED);
}


void MapCatalog::on_map_change(const msgs::String& message) {
  const std::string requested = message.data;
  if (requested.empty()) {
    std::string loaded;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      loaded = loaded_name_;
      loaded_name_.clear();
    }
    if (!loaded.empty() && safe_name(loaded)) {
      std::error_code error;
      std::filesystem::remove_all(directory() / loaded, error);
      if (error) {
        publish_status("delete failed: " + loaded + " (" + error.message() + ")");
      } else {
        publish_status("deleted " + loaded);
      }
    } else {
      publish_status("no map loaded");
    }
    try {
      call_service<msgs::Empty, msgs::Empty>(map_server_.value() + "/unload_map", msgs::Empty{});
    } catch (const std::exception& error) {
      logger().warn("map_server unload 실패: %s", error.what());
    }
    publish_slam_mode("idle");
    refresh();
    return;
  }
  if (!safe_name(requested)) {
    publish_status("invalid map name: " + requested);
    return;
  }
  const std::filesystem::path yaml = directory() / requested / "map.yaml";
  if (!std::filesystem::exists(yaml)) {
    publish_status("map not found: " + requested);
    return;
  }
  try {
    msgs::nav2::LoadMapRequest request;
    request.map_url = requested + "/map.yaml";
    const auto response =
        call_service<msgs::nav2::LoadMapRequest, msgs::nav2::LoadMapResponse>(
            map_server_.value() + "/load_map", request);
    if (response.result != msgs::nav2::LoadMapResponse::RESULT_SUCCESS) {
      publish_status("load failed: " + requested + " (" + std::to_string(response.result) + ")");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      loaded_name_ = requested;
    }
    publish_slam_mode("localization");
    publish_status("loaded " + requested);
    refresh();
  } catch (const std::exception& error) {
    publish_status("load failed: " + std::string(error.what()));
  }
}

void MapCatalog::on_map_unload(const msgs::Empty&) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    loaded_name_.clear();
  }
  try {
    call_service<msgs::Empty, msgs::Empty>(map_server_.value() + "/unload_map", msgs::Empty{});
  } catch (const std::exception& error) {
    logger().warn("map_server unload 실패: %s", error.what());
  }
  publish_slam_mode("idle");
  publish_status("map unloaded");
  refresh();
}

void MapCatalog::on_map_delete(const msgs::String& message) {
  const std::string name = message.data;
  if (!safe_name(name)) {
    publish_status("invalid map name: " + name);
    return;
  }
  bool was_loaded = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (loaded_name_ == name) {
      loaded_name_.clear();
      was_loaded = true;
    }
  }
  std::error_code error;
  const auto removed = std::filesystem::remove_all(directory() / name, error);
  if (error) {
    publish_status("delete failed: " + name + " (" + error.message() + ")");
    return;
  }
  if (removed == 0) {
    publish_status("map not found: " + name);
    return;
  }
  if (was_loaded) {
    try {
      call_service<msgs::Empty, msgs::Empty>(map_server_.value() + "/unload_map", msgs::Empty{});
    } catch (const std::exception& exception) {
      logger().warn("map_server unload 실패: %s", exception.what());
    }
    publish_slam_mode("idle");
  }
  publish_status("deleted " + name);
  refresh();
}

void MapCatalog::on_map_save(const msgs::String& message) {
  std::string name = message.data;
  if (name.empty()) {
    name = timestamp_name();
  }
  if (!safe_name(name)) {
    publish_status("invalid map name: " + name);
    return;
  }
  try {
    msgs::nav2::SaveMapRequest request;
    request.map_url = name + "/map";
    const auto response =
        call_service<msgs::nav2::SaveMapRequest, msgs::nav2::SaveMapResponse>(
            map_saver_.value() + "/save_map", request);
    if (!response.result) {
      publish_status("save failed: " + name);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      loaded_name_ = name;
    }
    publish_status("saved " + name);
    refresh();
  } catch (const std::exception& error) {
    publish_status("save failed: " + std::string(error.what()));
  }
}

}  // namespace krosc::nodes
