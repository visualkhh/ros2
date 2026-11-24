#include "krosc/nodes/nav2/regulated_pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>

namespace krosc::nodes::nav2 {

void RegulatedPurePursuitController::configure(Node& node, const std::string& name) {
  NavControllerPlugin::configure(node, name);
  const auto positive = [](double value) { return value > 0.0; };
  desired_linear_vel_ = node.declare_parameter<double>(name + ".desired_linear_vel", 0.22, positive);
  lookahead_dist_ = node.declare_parameter<double>(name + ".lookahead_dist", 0.6, positive);
  min_lookahead_dist_ = node.declare_parameter<double>(name + ".min_lookahead_dist", 0.3, positive);
  max_lookahead_dist_ = node.declare_parameter<double>(name + ".max_lookahead_dist", 0.9, positive);
  lookahead_time_ = node.declare_parameter<double>(name + ".lookahead_time", 1.5, positive);
  use_velocity_scaled_lookahead_dist_ =
      node.declare_parameter<bool>(name + ".use_velocity_scaled_lookahead_dist", false);
  use_rotate_to_heading_ = node.declare_parameter<bool>(name + ".use_rotate_to_heading", true);
  rotate_to_heading_angular_vel_ =
      node.declare_parameter<double>(name + ".rotate_to_heading_angular_vel", 1.8, positive);
  rotate_to_heading_min_angle_ =
      node.declare_parameter<double>(name + ".rotate_to_heading_min_angle", 0.785, positive);
  max_angular_accel_ = node.declare_parameter<double>(name + ".max_angular_accel", 3.2, positive);
  use_regulated_linear_velocity_scaling_ =
      node.declare_parameter<bool>(name + ".use_regulated_linear_velocity_scaling", true);
  regulated_linear_scaling_min_radius_ =
      node.declare_parameter<double>(name + ".regulated_linear_scaling_min_radius", 0.9, positive);
  regulated_linear_scaling_min_speed_ =
      node.declare_parameter<double>(name + ".regulated_linear_scaling_min_speed", 0.05, positive);
  approach_velocity_scaling_dist_ =
      node.declare_parameter<double>(name + ".approach_velocity_scaling_dist", 0.6, positive);
  min_approach_linear_velocity_ =
      node.declare_parameter<double>(name + ".min_approach_linear_velocity", 0.05, positive);
  use_collision_detection_ = node.declare_parameter<bool>(name + ".use_collision_detection", true);
  max_allowed_time_to_collision_up_to_carrot_ = node.declare_parameter<double>(
      name + ".max_allowed_time_to_collision_up_to_carrot", 1.0, positive);
  goal_dist_tol_ = node.declare_parameter<double>(name + ".goal_dist_tol", 0.2, positive);
}

double RegulatedPurePursuitController::rotate(double angle, double current, double dt) const {
  const double target = std::copysign(
      std::min(rotate_to_heading_angular_vel_.value(),
               std::sqrt(2.0 * max_angular_accel_.value() * std::abs(angle))),
      angle);
  const double step = max_angular_accel_.value() * dt;
  return std::min(current + step, std::max(current - step, target));
}

Velocity RegulatedPurePursuitController::compute_velocity_commands(const Pose2& pose,
                                                                  const Velocity& velocity,
                                                                  const std::vector<Pose2>& path,
                                                                  const Costmap& costmap, double dt) {
  const std::size_t start = closest_index(pose, path);
  const Pose2 goal = path.back();
  local_plan_.clear();
  carrot_.clear();
  if (std::hypot(goal.x - pose.x, goal.y - pose.y) < goal_dist_tol_.value()) {
    return Velocity{0.0, rotate(wrap_angle(goal.yaw - pose.yaw), velocity.angular, dt)};
  }
  double lookahead = lookahead_dist_.value();
  if (use_velocity_scaled_lookahead_dist_.value()) {
    lookahead = std::min(max_lookahead_dist_.value(),
                         std::max(min_lookahead_dist_.value(),
                                  std::abs(velocity.linear) * lookahead_time_.value()));
  }
  Pose2 carrot = path.back();
  for (std::size_t index = start; index < path.size(); ++index) {
    if (std::hypot(path[index].x - pose.x, path[index].y - pose.y) >= lookahead) {
      carrot = path[index];
      break;
    }
  }
  carrot_.push_back(carrot);
  const auto [cx, cy] = to_robot(pose, carrot.x, carrot.y);
  const double angle = std::atan2(cy, cx);
  if (use_rotate_to_heading_.value() && std::abs(angle) > rotate_to_heading_min_angle_.value()) {
    return Velocity{0.0, rotate(angle, velocity.angular, dt)};
  }
  const double distance2 = cx * cx + cy * cy;
  const double curvature = distance2 > 1e-9 ? 2.0 * cy / distance2 : 0.0;
  double v = desired_linear_vel_.value();
  if (use_regulated_linear_velocity_scaling_.value() && std::abs(curvature) > 1e-9) {
    const double radius = 1.0 / std::abs(curvature);
    if (radius < regulated_linear_scaling_min_radius_.value()) {
      v = std::max(v * radius / regulated_linear_scaling_min_radius_.value(),
                   regulated_linear_scaling_min_speed_.value());
    }
  }
  double remaining = std::hypot(path[start].x - pose.x, path[start].y - pose.y);
  for (std::size_t index = start; index + 1 < path.size(); ++index) {
    remaining += std::hypot(path[index + 1].x - path[index].x, path[index + 1].y - path[index].y);
  }
  if (remaining < approach_velocity_scaling_dist_.value()) {
    v = std::max(v * remaining / approach_velocity_scaling_dist_.value(),
                 min_approach_linear_velocity_.value());
  }
  const double w = v * curvature;
  const double seconds = std::min(max_allowed_time_to_collision_up_to_carrot_.value(),
                                  std::sqrt(distance2) / std::max(v, 1e-6));
  local_plan_ = simulate(pose, v, w, seconds, costmap.resolution / std::max(v, 1e-6));
  if (use_collision_detection_.value()) {
    for (const auto& point : local_plan_) {
      if (std::hypot(point.x - pose.x, point.y - pose.y) > 0.1 &&
          blocked(costmap, point.x, point.y)) {
        throw ControllerError(msgs::nav2::FollowPathResult::NO_VALID_CONTROL,
                              "앞 경로가 벽에 닿는다");
      }
    }
  }
  return Velocity{v, w};
}

}  // namespace krosc::nodes::nav2
