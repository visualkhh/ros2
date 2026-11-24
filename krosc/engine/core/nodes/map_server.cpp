#include "krosc/nodes/map_server.hpp"

#include <filesystem>
#include <utility>

#include "krosc/nodes/map_io.hpp"
#include "krosc/qos.hpp"

namespace krosc::nodes {

MapServer::MapServer(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      yaml_filename_(declare_parameter<std::string>("yaml_filename", "")),
      topic_name_(declare_parameter<std::string>("topic_name", "map")),
      frame_id_(declare_parameter<std::string>("frame_id", "map")),
      maps_directory_(declare_parameter<std::string>("maps_directory", "")) {
  load_service_ = create_service<msgs::nav2::LoadMapRequest, msgs::nav2::LoadMapResponse>(
      this->name() + "/load_map",
      [this](const msgs::nav2::LoadMapRequest& request) { return load(request.map_url); });
  unload_service_ = create_service<msgs::Empty, msgs::Empty>(
      this->name() + "/unload_map", [this](const msgs::Empty&) {
        unload();
        return msgs::Empty{};
      });
  if (!yaml_filename_.value().empty()) {
    const auto response = load(yaml_filename_.value());
    if (response.result != msgs::nav2::LoadMapResponse::RESULT_SUCCESS) {
      logger().error("시작 지도를 띄우지 못했다: %s", yaml_filename_.value().c_str());
    }
  }
  logger().info("map_server: topic=%s frame=%s maps=%s", topic_name_.value().c_str(),
                frame_id_.value().c_str(), maps_directory_.value().c_str());
}

std::string MapServer::resolve(const std::string& url) const {
  const std::filesystem::path path(url);
  if (path.is_absolute()) {
    return path.string();
  }
  return (std::filesystem::path(maps_directory_.value()) / path).string();
}

msgs::nav2::LoadMapResponse MapServer::load(const std::string& url) {
  msgs::nav2::LoadMapResponse response;
  const std::string path = resolve(url);
  std::error_code exists_error;
  if (!std::filesystem::exists(path, exists_error)) {
    response.result = msgs::nav2::LoadMapResponse::RESULT_MAP_DOES_NOT_EXIST;
    return response;
  }
  try {
    msgs::OccupancyGrid grid = load_map(path, frame_id_.value(), now());
    publish_grid(std::move(grid));
    response.result = msgs::nav2::LoadMapResponse::RESULT_SUCCESS;
  } catch (const std::filesystem::filesystem_error&) {
    response.result = msgs::nav2::LoadMapResponse::RESULT_MAP_DOES_NOT_EXIST;
  } catch (const MapIoError& error) {
    logger().error("지도를 읽지 못했다: %s", error.what());
    response.result = msgs::nav2::LoadMapResponse::RESULT_INVALID_MAP_DATA;
  } catch (const std::exception& error) {
    logger().error("지도를 읽지 못했다: %s", error.what());
    response.result = msgs::nav2::LoadMapResponse::RESULT_UNDEFINED_FAILURE;
  }
  return response;
}

void MapServer::unload() {
  msgs::OccupancyGrid empty;
  empty.header.stamp = now();
  empty.header.frame_id = frame_id_.value();
  empty.info.resolution = 0.05;
  empty.info.origin.orientation.w = 1.0;
  publish_grid(std::move(empty));
  logger().info("map_server: unloaded");
}

void MapServer::publish_grid(msgs::OccupancyGrid grid) {
  publish(topic_name_.value(), std::move(grid), QOS_LATCHED);
}

}  // namespace krosc::nodes
