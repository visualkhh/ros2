#include "krosc/nodes/map_catalog.hpp"

#include "krosc/msgs/nav2_srv.hpp"
#include "krosc/msgs/rcl_interfaces.hpp"

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
  map_list_topic_ = declare_parameter<std::string>("map_list_topic", std::string("map_list"));
  map_name_topic_ = declare_parameter<std::string>("map_name_topic", std::string("map_name"));
  map_status_topic_ = declare_parameter<std::string>("map_status_topic", std::string("map_status"));
  cartographer_node_ =
      declare_parameter<std::string>("cartographer_node", std::string("cartographer_node"));

  create_service<msgs::nav2::LoadMapRequest, msgs::nav2::LoadMapResponse>(
      this->name() + "/load_map", [this](const msgs::nav2::LoadMapRequest& request) { return on_load(request); });
  create_service<msgs::nav2::SaveMapRequest, msgs::nav2::SaveMapResponse>(
      this->name() + "/save_map", [this](const msgs::nav2::SaveMapRequest& request) { return on_save(request); });
  create_service<msgs::nav2::DeleteMapRequest, msgs::nav2::DeleteMapResponse>(
      this->name() + "/delete_map", [this](const msgs::nav2::DeleteMapRequest& request) { return on_delete(request); });
  create_service<msgs::Empty, msgs::Empty>(
      this->name() + "/unload_map", [this](const msgs::Empty& request) { return on_unload(request); });
  create_service<msgs::String, msgs::Bool>(
      this->name() + "/set_slam_mode", [this](const msgs::String& request) { return on_set_slam_mode(request); });
  create_timer(list_period_.value(), [this] { refresh(); });
  refresh();
  publish_status("ready");
  publish_map_name();
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

void MapCatalog::publish_map_name() {
  publish(map_name_topic_.value(), msgs::String{loaded_name()}, QOS_LATCHED);
}

void MapCatalog::publish_status(const std::string& text) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = text;
  }
  publish(map_status_topic_.value(), msgs::String{text}, QOS_LATCHED);
}

bool MapCatalog::set_slam_mode(const std::string& mode) {
  msgs::rcl_interfaces::SetParametersRequest request;
  msgs::rcl_interfaces::Parameter parameter;
  parameter.name = "mode";
  parameter.value.type =
      static_cast<std::uint8_t>(msgs::rcl_interfaces::ParameterType::String);
  parameter.value.string_value = mode;
  request.parameters.push_back(std::move(parameter));
  try {
    const auto response = call_service<msgs::rcl_interfaces::SetParametersRequest,
                                       msgs::rcl_interfaces::SetParametersResponse>(
        cartographer_node_.value() + "/set_parameters", request);
    if (!response.results.empty() && !response.results.front().successful) {
      logger().warn("Cartographer mode=%s 실패: %s", mode.c_str(),
                    response.results.front().reason.c_str());
      return false;
    }
    return true;
  } catch (const std::exception& error) {
    logger().warn("Cartographer mode=%s 실패: %s", mode.c_str(), error.what());
    return false;
  }
}


msgs::nav2::LoadMapResponse MapCatalog::on_load(const msgs::nav2::LoadMapRequest& request) {
  msgs::nav2::LoadMapResponse response;
  std::string name = request.map_url;
  const std::string suffix = "/map.yaml";
  if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
    name = name.substr(0, name.size() - suffix.size());
  }
  if (!safe_name(name)) {
    publish_status("invalid map name: " + name);
    response.result = msgs::nav2::LoadMapResponse::RESULT_INVALID_MAP_DATA;
    return response;
  }
  const std::filesystem::path yaml = directory() / name / "map.yaml";
  if (!std::filesystem::exists(yaml)) {
    publish_status("map not found: " + name);
    response.result = msgs::nav2::LoadMapResponse::RESULT_MAP_DOES_NOT_EXIST;
    return response;
  }
  try {
    msgs::nav2::LoadMapRequest load;
    load.map_url = name + "/map.yaml";
    response = call_service<msgs::nav2::LoadMapRequest, msgs::nav2::LoadMapResponse>(
        map_server_.value() + "/load_map", load);
    if (response.result != msgs::nav2::LoadMapResponse::RESULT_SUCCESS) {
      publish_status("load failed: " + name + " (" + std::to_string(response.result) + ")");
      return response;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      loaded_name_ = name;
    }
    set_slam_mode("localization");
    publish_map_name();
    publish_status("loaded " + name);
    refresh();
  } catch (const std::exception& error) {
    response.result = msgs::nav2::LoadMapResponse::RESULT_UNDEFINED_FAILURE;
    publish_status("load failed: " + std::string(error.what()));
  }
  return response;
}

msgs::nav2::SaveMapResponse MapCatalog::on_save(const msgs::nav2::SaveMapRequest& request) {
  msgs::nav2::SaveMapResponse response;
  std::string name = request.map_url;
  const std::string suffix = "/map";
  if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
    name = name.substr(0, name.size() - suffix.size());
  }
  if (name.empty()) {
    name = timestamp_name();
  }
  if (!safe_name(name)) {
    publish_status("invalid map name: " + name);
    return response;
  }
  try {
    msgs::nav2::SaveMapRequest save = request;
    save.map_url = name + "/map";
    response = call_service<msgs::nav2::SaveMapRequest, msgs::nav2::SaveMapResponse>(
        map_saver_.value() + "/save_map", save);
    if (!response.result) {
      publish_status("save failed: " + name);
      return response;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      loaded_name_ = name;
    }
    publish_map_name();
    publish_status("saved " + name);
    refresh();
  } catch (const std::exception& error) {
    publish_status("save failed: " + std::string(error.what()));
  }
  return response;
}

msgs::nav2::DeleteMapResponse MapCatalog::on_delete(const msgs::nav2::DeleteMapRequest& request) {
  msgs::nav2::DeleteMapResponse response;
  std::string name = request.map_url;
  const std::string suffix = "/map.yaml";
  if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
    name = name.substr(0, name.size() - suffix.size());
  }
  if (!safe_name(name)) {
    response.message = "invalid map name: " + name;
    publish_status(response.message);
    return response;
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
    response.message = "delete failed: " + name + " (" + error.message() + ")";
    publish_status(response.message);
    return response;
  }
  if (removed == 0) {
    response.message = "map not found: " + name;
    publish_status(response.message);
    return response;
  }
  if (was_loaded) {
    try {
      call_service<msgs::Empty, msgs::Empty>(map_server_.value() + "/unload_map", msgs::Empty{});
    } catch (const std::exception& exception) {
      logger().warn("map_server unload 실패: %s", exception.what());
    }
    set_slam_mode("idle");
    publish_map_name();
  }
  response.result = true;
  response.message = "deleted " + name;
  publish_status(response.message);
  refresh();
  return response;
}

msgs::Empty MapCatalog::on_unload(const msgs::Empty&) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    loaded_name_.clear();
  }
  try {
    call_service<msgs::Empty, msgs::Empty>(map_server_.value() + "/unload_map", msgs::Empty{});
  } catch (const std::exception& error) {
    logger().warn("map_server unload 실패: %s", error.what());
  }
  set_slam_mode("idle");
  publish_map_name();
  publish_status("map unloaded");
  refresh();
  return msgs::Empty{};
}

msgs::Bool MapCatalog::on_set_slam_mode(const msgs::String& request) {
  const std::string& mode = request.data;
  if (mode != "mapping" && mode != "localization" && mode != "idle") {
    publish_status("invalid slam mode: " + mode);
    return msgs::Bool{false};
  }
  const bool applied = set_slam_mode(mode);
  publish_status(applied ? "slam mode: " + mode : "slam mode 실패: " + mode);
  return msgs::Bool{applied};
}

}  // namespace krosc::nodes
