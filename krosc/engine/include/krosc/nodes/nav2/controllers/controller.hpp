#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/node.hpp"

#include "krosc/nodes/nav2/costmap/costmap.hpp"
#include "krosc/nodes/nav2/planners/global_planner.hpp"

namespace krosc::nodes::nav2 {

struct Velocity {
  double linear = 0.0;
  double angular = 0.0;
};

class ControllerError : public std::runtime_error {
 public:
  ControllerError(int error_code, std::string message) : std::runtime_error(std::move(message)), code(error_code) {}
  int code = msgs::nav2::FollowPathResult::UNKNOWN;
};

inline double wrap_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

inline std::size_t closest_index(const Pose2& pose, const std::vector<Pose2>& path) {
  if (path.empty()) {
    return 0;
  }
  std::size_t best = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double distance = std::pow(path[index].x - pose.x, 2.0) + std::pow(path[index].y - pose.y, 2.0);
    if (distance < best_distance) {
      best_distance = distance;
      best = index;
    }
  }
  return best;
}

inline std::pair<double, double> to_robot(const Pose2& pose, double x, double y) {
  const double dx = x - pose.x;
  const double dy = y - pose.y;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {c * dx + s * dy, -s * dx + c * dy};
}

inline bool blocked(const Costmap& costmap, double x, double y) {
  return costmap.cost_at(x, y) >= INSCRIBED_INFLATED_OBSTACLE;
}

inline double distance_to_path(double x, double y, const std::vector<Pose2>& path) {
  double best = std::numeric_limits<double>::infinity();
  for (const auto& point : path) {
    best = std::min(best, std::hypot(x - point.x, y - point.y));
  }
  return best;
}

inline std::vector<Pose2> simulate(const Pose2& pose, double linear, double angular, double seconds,
                                   double step) {
  std::vector<Pose2> points;
  const int count = std::max(1, static_cast<int>(std::lround(seconds / step)));
  double x = pose.x;
  double y = pose.y;
  double yaw = pose.yaw;
  for (int index = 0; index < count; ++index) {
    x += linear * std::cos(yaw) * step;
    y += linear * std::sin(yaw) * step;
    yaw += angular * step;
    points.push_back(Pose2{x, y, yaw});
  }
  return points;
}

class ControllerRegistry;

class NavControllerPlugin {
 public:
  virtual ~NavControllerPlugin() = default;
  // pluginlib 대신 정적 registry를 쓰므로, primary controller를 만드는 플러그인
  // (rotation shim)이 서버의 registry를 볼 수 있게 넘긴다.
  void set_registry(const ControllerRegistry* registry) { registry_ = registry; }
  // nav2_core::Controller::configure(parent, ...)와 같이 '서버'가 아니라 parent Node를 받는다.
  // ControllerServer는 자기 자신(*this)을 Node&로 넘긴다 (복사/슬라이싱 없음).
  virtual void configure(Node& node, const std::string& name) {
    node_ = &node;
    name_ = name;
    local_plan_.clear();
  }
  virtual Velocity compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                             const std::vector<Pose2>& path, const Costmap& costmap,
                                             double dt) = 0;
  [[nodiscard]] const std::vector<Pose2>& local_plan() const { return local_plan_; }
  [[nodiscard]] virtual std::string type() const = 0;

 protected:
  Node* node_ = nullptr;
  std::string name_;
  std::vector<Pose2> local_plan_;
  const ControllerRegistry* registry_ = nullptr;
};

using ControllerFactory = std::function<std::unique_ptr<NavControllerPlugin>()>;

class ControllerRegistry {
 public:
  void add(const std::string& name, ControllerFactory factory) { factories_[name] = std::move(factory); }
  [[nodiscard]] std::unique_ptr<NavControllerPlugin> create(const std::string& name) const {
    auto it = factories_.find(name);
    return it == factories_.end() ? nullptr : it->second();
  }
  [[nodiscard]] bool contains(const std::string& name) const { return factories_.count(name) != 0; }
  [[nodiscard]] std::vector<std::string> names() const {
    std::vector<std::string> result;
    for (const auto& [name, factory] : factories_) {
      (void)factory;
      result.push_back(name);
    }
    return result;
  }

 private:
  std::map<std::string, ControllerFactory> factories_;
};

ControllerRegistry builtin_controllers();

// 서버 없이 플러그인만 직접 만들 때 쓰는 기본 registry (테스트/단독 사용).
inline const ControllerRegistry& builtin_controller_registry() {
  static const ControllerRegistry registry = builtin_controllers();
  return registry;
}

}  // namespace krosc::nodes::nav2
