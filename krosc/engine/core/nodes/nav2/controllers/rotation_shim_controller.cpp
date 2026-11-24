#include "krosc/nodes/nav2/controllers/rotation_shim_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "krosc/math.hpp"

namespace krosc::nodes::nav2 {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

void RotationShimController::configure(Node& node, const std::string& name) {
  NavControllerPlugin::configure(node, name);
  const auto positive = [](double value) { return value > 0.0; };
  const auto prefix = [&name](const char* suffix) { return name + "." + suffix; };

  angular_dist_threshold_ = node_->declare_parameter<double>(
      prefix("angular_dist_threshold"), 0.785,
      [](double value) { return value > 0.0 && value <= kPi; });
  angular_disengage_threshold_ = node_->declare_parameter<double>(
      prefix("angular_disengage_threshold"), 0.785 / 2.0,
      [](double value) { return value > 0.0 && value <= kPi; });
  forward_sampling_distance_ =
      node_->declare_parameter<double>(prefix("forward_sampling_distance"), 0.5, positive);
  rotate_to_heading_angular_vel_ = node_->declare_parameter<double>(
      prefix("rotate_to_heading_angular_vel"), 1.8, positive);
  max_angular_accel_ =
      node_->declare_parameter<double>(prefix("max_angular_accel"), 3.2, positive);
  max_cost_threshold_ = node_->declare_parameter<double>(
      prefix("max_cost_threshold"), static_cast<double>(LETHAL_OBSTACLE),
      [](double value) { return value >= 0.0 && value <= 255.0; });
  simulate_ahead_time_ =
      node_->declare_parameter<double>(prefix("simulate_ahead_time"), 1.0, positive);
  use_path_orientations_ =
      node_->declare_parameter<bool>(prefix("use_path_orientations"), false);
  closed_loop_ = node_->declare_parameter<bool>(prefix("closed_loop"), true);
  primary_plugin_ = node_->declare_parameter<std::string>(
      prefix("primary_controller.plugin"),
      "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
      [this](const std::string& value) {
        const ControllerRegistry& registry =
            registry_ != nullptr ? *registry_ : builtin_controller_registry();
        return registry.contains(value);
      });

  const ControllerRegistry& registry =
      registry_ != nullptr ? *registry_ : builtin_controller_registry();
  primary_ = registry.create(primary_plugin_.value());
  if (!primary_) {
    throw std::runtime_error("rotation shim primary controller not found: " +
                             primary_plugin_.value());
  }
  primary_->set_registry(&registry);
  primary_->configure(*node_, name_ + ".primary_controller");
  in_rotation_ = false;
  last_angular_vel_ = 0.0;
}

RotationShimController::Sample RotationShimController::sample_path(
    const std::vector<Pose2>& path) const {
  Sample sample;
  if (path.size() < 2) {
    return sample;
  }
  const auto& start = path.front();
  const double distance = forward_sampling_distance_.value();
  for (std::size_t index = 1; index < path.size(); ++index) {
    if (std::hypot(path[index].x - start.x, path[index].y - start.y) >= distance) {
      sample.x = path[index].x;
      sample.y = path[index].y;
      sample.yaw = path[index].yaw;
      sample.valid = true;
      return sample;
    }
  }
  sample.x = path.back().x;
  sample.y = path.back().y;
  sample.yaw = path.back().yaw;
  sample.valid = true;
  return sample;
}

Velocity RotationShimController::rotate_to_heading(
    double angular_distance, const Pose2& pose, const Velocity& velocity, const Costmap& costmap,
    double dt) {
  const double current = closed_loop_.value() ? velocity.angular : last_angular_vel_;
  const double sign = angular_distance > 0.0 ? 1.0 : -1.0;
  const double step = std::max(dt, 1e-3);
  const double target = sign * rotate_to_heading_angular_vel_.value();
  const double min_feasible = current - max_angular_accel_.value() * step;
  const double max_feasible = current + max_angular_accel_.value() * step;
  double angular = std::clamp(target, min_feasible, max_feasible);

  // 목표 각도에 가까우면 미리 감속해서 지나치지 않게 한다.
  const double max_to_stop =
      std::sqrt(2.0 * max_angular_accel_.value() * std::abs(angular_distance));
  if (std::abs(angular) > max_to_stop) {
    angular = sign * max_to_stop;
  }

  check_collision(costmap, pose, angular, angular_distance, step);
  last_angular_vel_ = angular;
  in_rotation_ = true;
  local_plan_ = simulate(pose, 0.0, angular, 1.0, 0.1);
  return Velocity{0.0, angular};
}

void RotationShimController::check_collision(const Costmap& costmap, const Pose2& pose,
                                             double angular_velocity, double angular_distance,
                                             double dt) const {
  const double remaining =
      std::max(0.0, std::abs(angular_distance) - angular_dist_threshold_.value());
  double simulated = 0.0;
  while (simulated < simulate_ahead_time_.value()) {
    simulated += dt;
    const double yaw = pose.yaw + angular_velocity * simulated;
    if (std::abs(wrap_angle(yaw - pose.yaw)) >= remaining) {
      break;
    }
    if (costmap.cost_at(pose.x, pose.y) >= max_cost_threshold_.value()) {
      throw ControllerError(msgs::nav2::FollowPathResult::NO_VALID_CONTROL,
                            "RotationShimController: 회전 경로에 장애물이 있다");
    }
  }
}

Velocity RotationShimController::compute_velocity_commands(
    const Pose2& pose, const Velocity& velocity, const std::vector<Pose2>& path,
    const Costmap& costmap, double dt) {
  if (!primary_) {
    throw ControllerError(msgs::nav2::FollowPathResult::NO_VALID_CONTROL,
                          "RotationShimController: primary controller가 없다");
  }

  const Sample sample = sample_path(path);
  if (sample.valid) {
    const auto relative = to_robot(pose, sample.x, sample.y);
    const double angular_distance =
        use_path_orientations_.value() ? wrap_angle(sample.yaw - pose.yaw)
                                       : std::atan2(relative.second, relative.first);
    const double threshold = in_rotation_ ? angular_disengage_threshold_.value()
                                          : angular_dist_threshold_.value();
    if (std::abs(angular_distance) > threshold) {
      return rotate_to_heading(angular_distance, pose, velocity, costmap, dt);
    }
  }

  in_rotation_ = false;
  const Velocity command =
      primary_->compute_velocity_commands(pose, velocity, path, costmap, dt);
  last_angular_vel_ = command.angular;
  local_plan_ = primary_->local_plan();
  return command;
}

}  // namespace krosc::nodes::nav2
