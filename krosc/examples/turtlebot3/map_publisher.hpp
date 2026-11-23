#pragma once

#include <string>
#include <vector>

#include "krosc/node.hpp"
#include "krosc/sim/world.hpp"
#include "krosc/tf_ros.hpp"

namespace krosc::examples {

// MazeWorld의 벽을 nav_msgs/OccupancyGrid로 만들어 map 토픽(latch)에 낸다.
// 데모에서는 SLAM 대신 지도를 미리 알고 있다고 보고 map -> odom 정적 TF도 낸다.
class MapPublisher final : public krosc::Node {
 public:
  MapPublisher(krosc::Context& context, const std::vector<krosc::sim::Wall>& walls, std::string name = "MapPublisher");

 private:
  void publish_map(const std::vector<krosc::sim::Wall>& walls);

  krosc::StaticTransformBroadcaster static_tf_;
};

}  // namespace krosc::examples
