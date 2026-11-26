#include "map_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"

namespace krosc::simulator {

namespace {
constexpr double kResolution = 0.05;
constexpr double kMargin = 0.25;
}

MapPublisher::MapPublisher(krosc::Context& context, const std::vector<krosc::simulator::Wall>& walls, std::string name)
    : Node(context, std::move(name)), static_tf_(*this) {
  publish_map(walls);
  // 데모는 미리 아는 지도 + map→odom 정적 TF를 쓰므로 localization 상태로 알린다.
  // (Viz/웹은 slam_status가 idle이면 래치된 옛 지도를 숨긴다.)
  publish("slam_status", krosc::msgs::String{"localization"}, krosc::QOS_LATCHED);
  krosc::msgs::TransformStamped map_to_odom;
  map_to_odom.header.stamp = now();
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform.rotation.w = 1.0;
  static_tf_.send_transform(map_to_odom);
}

void MapPublisher::publish_map(const std::vector<krosc::simulator::Wall>& walls) {
  if (walls.empty()) {
    return;
  }
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto& wall : walls) {
    min_x = std::min(min_x, wall.center_x - wall.half_x);
    min_y = std::min(min_y, wall.center_y - wall.half_y);
    max_x = std::max(max_x, wall.center_x + wall.half_x);
    max_y = std::max(max_y, wall.center_y + wall.half_y);
  }
  min_x -= kMargin;
  min_y -= kMargin;
  max_x += kMargin;
  max_y += kMargin;

  const int width = static_cast<int>(std::ceil((max_x - min_x) / kResolution));
  const int height = static_cast<int>(std::ceil((max_y - min_y) / kResolution));
  krosc::msgs::OccupancyGrid grid;
  grid.header.stamp = now();
  grid.header.frame_id = "map";
  grid.info.map_load_time = now();
  grid.info.resolution = kResolution;
  grid.info.width = static_cast<std::uint32_t>(width);
  grid.info.height = static_cast<std::uint32_t>(height);
  grid.info.origin.position.x = min_x;
  grid.info.origin.position.y = min_y;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width * height), 0);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double px = min_x + (x + 0.5) * kResolution;
      const double py = min_y + (y + 0.5) * kResolution;
      bool occupied = false;
      bool inside_room = false;
      for (const auto& wall : walls) {
        const double lo_x = wall.center_x - wall.half_x;
        const double hi_x = wall.center_x + wall.half_x;
        const double lo_y = wall.center_y - wall.half_y;
        const double hi_y = wall.center_y + wall.half_y;
        if (px >= lo_x && px <= hi_x && py >= lo_y && py <= hi_y) {
          occupied = true;
        }
        if (px > lo_x && px < hi_x && py > lo_y && py < hi_y) {
          inside_room = true;
        }
      }
      // 방 바깥(벽도 아니고 안쪽도 아님)은 모르는 칸으로 둔다.
      grid.data[static_cast<std::size_t>(y * width + x)] =
          occupied ? static_cast<std::int8_t>(100)
                   : (inside_room || (px > min_x + kMargin - kResolution && px < max_x - kMargin + kResolution &&
                                      py > min_y + kMargin - kResolution && py < max_y - kMargin + kResolution)
                          ? static_cast<std::int8_t>(0)
                          : static_cast<std::int8_t>(-1));
    }
  }
  publish("map", std::move(grid), krosc::QOS_LATCHED);
  logger().info("지도: %dx%d (%.1f x %.1f m, resolution %.2f)", width, height, width * kResolution,
                height * kResolution, kResolution);
}

}  // namespace krosc::simulator
