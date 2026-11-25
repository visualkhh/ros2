#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/tf.hpp"
#include "krosc/tf_ros.hpp"

namespace krosc::nodes {

// nav2 collision_monitor의 C++ 최소 버전.
//   cmd_vel_in을 라이다 점 + 로봇 주변 polygon/circle로 검사해 cmd_vel_out으로 낸다.
//   지도에 없는 장애물에도 반응하는 마지막 안전장치다.
class CollisionMonitor final : public Node {
 public:
  explicit CollisionMonitor(Context& context, std::string name = "collision_monitor",
                            std::map<std::string, ParamValue> parameters = {});
  ~CollisionMonitor() override = default;

  CollisionMonitor(const CollisionMonitor&) = delete;
  CollisionMonitor& operator=(const CollisionMonitor&) = delete;

 public:
  enum class Action { Stop, Slowdown, Limit, Approach };

 private:

  struct Polygon {
    std::string name;
    std::string type = "polygon";
    std::vector<std::pair<double, double>> points;
    double radius = 0.3;
    Action action = Action::Stop;
    std::int64_t min_points = 4;
    double slowdown_ratio = 0.3;
    double linear_limit = 0.1;
    double angular_limit = 0.5;
    bool enabled = true;

    [[nodiscard]] bool contains(double x, double y) const;
    [[nodiscard]] bool hits(const std::vector<std::pair<double, double>>& points) const;
    [[nodiscard]] std::pair<double, double> apply(double linear, double angular,
                                                  const std::vector<std::pair<double, double>>& points) const;
  };

  struct Observation {
    double stamp = 0.0;
    std::vector<std::pair<double, double>> points;
  };

  void on_scan(const std::string& source, const msgs::LaserScan& scan);
  void on_cmd_vel(const msgs::Twist& twist);
  void on_teleop(const msgs::Twist& twist);
  void on_enabled(bool enabled);
  [[nodiscard]] std::pair<double, double> check(double linear, double angular);
  void publish_status();

  Parameter<std::string> base_frame_id_;
  Parameter<std::string> cmd_vel_in_topic_;
  Parameter<std::string> cmd_vel_out_topic_;
  Parameter<std::string> teleop_topic_;
  Parameter<std::string> status_topic_;
  Parameter<double> source_timeout_;
  Parameter<double> teleop_timeout_;
  Parameter<double> max_observation_range_;
  Parameter<double> reverse_escape_ratio_;
  Parameter<double> rotation_clearance_;
  Parameter<double> rotation_escape_ratio_;
  Parameter<bool> enabled_;
  Parameter<bool> ignore_collision_during_teleop_;
  Parameter<std::vector<std::string>> polygon_names_;
  Parameter<std::vector<std::string>> observation_sources_;

  std::vector<Polygon> polygons_;
  Buffer buffer_;
  std::unique_ptr<TransformListener> listener_;
  mutable std::mutex mutex_;
  std::map<std::string, Observation> observations_;
  double last_teleop_ = -1e18;
  std::string state_;
  std::string last_status_;
  std::vector<Subscription<msgs::LaserScan>> scan_subscriptions_;
  Subscription<msgs::Twist> command_subscription_;
  Subscription<msgs::Twist> teleop_subscription_;
};

}  // namespace krosc::nodes
