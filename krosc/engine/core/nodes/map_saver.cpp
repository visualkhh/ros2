#include "krosc/nodes/map_saver.hpp"

#include <chrono>
#include <filesystem>
#include <thread>
#include <utility>

#include "krosc/nodes/map_io.hpp"

namespace krosc::nodes {

MapSaver::MapSaver(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      save_map_timeout_(declare_parameter<double>("save_map_timeout", 2.0,
                                                  [](double value) { return value > 0.0; })),
      free_thresh_default_(declare_parameter<double>("free_thresh_default", 0.196,
                                                     [](double value) { return value > 0.0 && value < 1.0; })),
      occupied_thresh_default_(declare_parameter<double>("occupied_thresh_default", 0.65,
                                                         [](double value) { return value > 0.0 && value < 1.0; })),
      maps_directory_(declare_parameter<std::string>("maps_directory", "")) {
  save_service_ = create_service<msgs::nav2::SaveMapRequest, msgs::nav2::SaveMapResponse>(
      this->name() + "/save_map",
      [this](const msgs::nav2::SaveMapRequest& request) {
        msgs::nav2::SaveMapResponse response;
        response.result = save(request);
        return response;
      });
  logger().info("map_saver: maps=%s", maps_directory_.value().c_str());
}

std::string MapSaver::resolve(const std::string& url) const {
  std::filesystem::path path(url.empty() ? "map" : url);
  if (path.extension() == ".yaml") {
    path.replace_extension();
  }
  if (path.is_absolute()) {
    return path.string();
  }
  return (std::filesystem::path(maps_directory_.value()) / path).string();
}

bool MapSaver::save(const msgs::nav2::SaveMapRequest& request) {
  if (request.image_format != "pgm" || request.map_mode != "trinary") {
    logger().error("pgm/trinary만 저장한다: %s/%s", request.image_format.c_str(),
                   request.map_mode.c_str());
    return false;
  }
  std::mutex mutex;
  std::condition_variable arrived;
  std::shared_ptr<const msgs::OccupancyGrid> map;
  Subscription<msgs::OccupancyGrid> subscription = subscribe<msgs::OccupancyGrid>(
      request.map_topic.empty() ? "map" : request.map_topic,
      [&](const msgs::OccupancyGrid& grid) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          map = std::make_shared<const msgs::OccupancyGrid>(grid);
        }
        arrived.notify_all();
      },
      QOS_LATCHED);
  {
    std::unique_lock<std::mutex> lock(mutex);
    if (!arrived.wait_for(lock, std::chrono::duration<double>(save_map_timeout_.value()),
                          [&] { return map != nullptr; })) {
      logger().error("map 토픽을 %0.1f초 안에 받지 못했다", save_map_timeout_.value());
      subscription.cancel();
      return false;
    }
  }
  subscription.cancel();
  const double free = request.free_thresh > 0.0 ? request.free_thresh : free_thresh_default_.value();
  const double occupied = request.occupied_thresh > 0.0 ? request.occupied_thresh
                                                        : occupied_thresh_default_.value();
  try {
    const std::string path = save_map(*map, resolve(request.map_url), free, occupied);
    logger().info("지도 저장: %s", path.c_str());
    return true;
  } catch (const std::exception& error) {
    logger().error("지도 저장 실패: %s", error.what());
    return false;
  }
}

}  // namespace krosc::nodes
