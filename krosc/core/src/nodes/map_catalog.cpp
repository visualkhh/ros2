#include "krosc/nodes/map_catalog.hpp"

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <sstream>
#include <string>
#include <utility>

#include "krosc/nodes/map_io.hpp"
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
  map_topic_ = declare_parameter<std::string>("map_topic", std::string("map"));
  map_change_topic_ = declare_parameter<std::string>("map_change_topic", std::string("map_change"));
  map_save_topic_ = declare_parameter<std::string>("map_save_topic", std::string("map_save"));
  map_unload_topic_ = declare_parameter<std::string>("map_unload_topic", std::string("map_unload"));
  map_list_topic_ = declare_parameter<std::string>("map_list_topic", std::string("map_list"));
  map_status_topic_ = declare_parameter<std::string>("map_status_topic", std::string("map_status"));
  slam_mode_topic_ = declare_parameter<std::string>("slam_mode_topic", std::string("slam_mode"));

  subscribe<msgs::OccupancyGrid>(
      map_topic_.value(), [this](const msgs::OccupancyGrid& grid) { on_map(grid); }, QOS_LATCHED);
  subscribe<msgs::String>(
      map_change_topic_.value(), [this](const msgs::String& message) { on_map_change(message); });
  subscribe<msgs::String>(
      map_save_topic_.value(), [this](const msgs::String& message) { on_map_save(message); });
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

void MapCatalog::on_map(const msgs::OccupancyGrid& grid) {
  std::lock_guard<std::mutex> lock(mutex_);
  map_ = grid;
}

void MapCatalog::on_map_change(const msgs::String& message) {
  const std::string requested = message.data;
  if (requested.empty()) {
    std::string loaded;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      loaded = loaded_name_;
      loaded_name_.clear();
      map_.reset();
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
    msgs::OccupancyGrid empty;
    empty.header.stamp = now();
    empty.header.frame_id = map_frame_.value();
    empty.info.resolution = 0.05;
    empty.info.origin.orientation.w = 1.0;
    publish(map_topic_.value(), std::move(empty), QOS_LATCHED);
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
    const msgs::OccupancyGrid grid = load_map(yaml.string(), map_frame_.value(), now());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      loaded_name_ = requested;
      map_ = grid;
    }
    publish(map_topic_.value(), grid, QOS_LATCHED);
    publish_slam_mode("localization");
    publish_status("loaded " + requested);
    refresh();
  } catch (const MapIoError& error) {
    publish_status("load failed: " + std::string(error.what()));
  } catch (const std::exception& error) {
    publish_status("load failed: " + std::string(error.what()));
  }
}

void MapCatalog::on_map_unload(const msgs::Empty&) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    loaded_name_.clear();
    map_.reset();
  }
  msgs::OccupancyGrid empty;
  empty.header.stamp = now();
  empty.header.frame_id = map_frame_.value();
  empty.info.resolution = 0.05;
  empty.info.origin.orientation.w = 1.0;
  publish(map_topic_.value(), std::move(empty), QOS_LATCHED);
  publish_slam_mode("idle");
  publish_status("map unloaded");
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
  msgs::OccupancyGrid grid;
  bool has_map = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_map = map_ && map_->info.width != 0 && map_->info.height != 0;
    if (has_map) {
      grid = *map_;
    }
  }
  if (!has_map) {
    publish_status("save failed: no map yet (start SLAM first)");
    return;
  }
  if (grid.header.frame_id.empty()) {
    grid.header.frame_id = map_frame_.value();
  }
  try {
    const std::string yaml = save_map(grid, map_path(name).string());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      loaded_name_ = name;
    }
    publish_status("saved " + name + " (" + yaml + ")");
    refresh();
  } catch (const std::exception& error) {
    publish_status("save failed: " + std::string(error.what()));
  }
}

}  // namespace krosc::nodes
