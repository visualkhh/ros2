#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/tf.hpp"
#include "krosc/tf_ros.hpp"
#include "krosc/timer.hpp"
#include "krosc/topic.hpp"

#include "krosc/nodes/nav2/costmap/costmap.hpp"

namespace krosc::nodes::nav2 {

// nav2_costmap_2d::Costmap2DROS 흉내.
//   플러그인 레이어(static/obstacle/inflation)를 겹쳐 costmap을 만들고 "<name>/costmap"(OccupancyGrid, latch)으로 낸다.
//   global_costmap은 map 기준 지도 크기, local_costmap은 odom 기준 로봇을 따라다니는 rolling window.
//   planner/controller는 이 노드의 costmap()을 읽는다 (별도 노드로 분리).
class Costmap2DROS final : public Node {
 public:
  explicit Costmap2DROS(Context& context, std::string name = "global_costmap",
                        std::map<std::string, ParamValue> parameters = {});

  [[nodiscard]] std::shared_ptr<const Costmap> costmap() const;
  [[nodiscard]] std::optional<msgs::Pose2D> get_robot_pose() const;
  [[nodiscard]] Buffer& buffer() { return buffer_; }
  [[nodiscard]] std::string global_frame() const { return global_frame_.value(); }
  [[nodiscard]] std::string robot_base_frame() const { return robot_base_frame_.value(); }

 private:
  void on_map(const msgs::OccupancyGrid& map);
  void on_scan(const std::string& source, const msgs::LaserScan& scan);
  void rebuild();
  void publish_costmap();
  [[nodiscard]] bool has_layer(const std::string& layer) const;
  [[nodiscard]] double costmap_resolution() const;

  Parameter<std::string> global_frame_;
  Parameter<std::string> robot_base_frame_;
  Parameter<std::vector<std::string>> plugins_;
  Parameter<double> robot_radius_;
  Parameter<double> inflation_radius_;
  Parameter<double> cost_scaling_factor_;
  Parameter<bool> track_unknown_space_;
  Parameter<std::string> map_topic_;
  Parameter<bool> rolling_window_;
  Parameter<double> width_;
  Parameter<double> height_;
  Parameter<double> resolution_;
  Parameter<double> update_frequency_;
  Parameter<double> scan_period_;
  Parameter<std::vector<std::string>> observation_sources_;
  Parameter<double> obstacle_max_range_;
  Parameter<double> raytrace_max_range_;

  Buffer buffer_;
  std::unique_ptr<TransformListener> listener_;
  mutable std::mutex mutex_;
  std::optional<msgs::OccupancyGrid> map_;
  std::shared_ptr<Costmap> costmap_;
  std::set<std::pair<int, int>> obstacles_;
  double last_scan_time_ = -1e18;
  std::vector<Subscription<msgs::LaserScan>> scan_subscriptions_;
  Subscription<msgs::OccupancyGrid> map_subscription_;
  Timer update_timer_;
};

}  // namespace krosc::nodes::nav2
