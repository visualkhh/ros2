#include "krosc/nodes/nav2/dwb_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace krosc::nodes::nav2 {

namespace {

std::vector<double> samples(double low, double high, std::int64_t count) {
  std::vector<double> values;
  if (high <= low || count <= 1) {
    values.push_back(low <= 0.0 && 0.0 <= high ? 0.0 : low);
    return values;
  }
  values.reserve(static_cast<std::size_t>(count));
  for (std::int64_t k = 0; k < count; ++k) {
    values.push_back(low + (high - low) * static_cast<double>(k) / static_cast<double>(count - 1));
  }
  return values;
}

}  // namespace

// ---------------------------------------------------------------- DWB

void DWBLocalPlanner::configure(Node& node, const std::string& name) {
  NavControllerPlugin::configure(node, name);
  const auto positive = [](double value) { return value > 0.0; };
  const auto non_negative = [](double value) { return value >= 0.0; };
  min_vel_x_ = node.declare_parameter<double>(name + ".min_vel_x", 0.0, non_negative);
  max_vel_x_ = node.declare_parameter<double>(name + ".max_vel_x", 0.22, positive);
  max_vel_theta_ = node.declare_parameter<double>(name + ".max_vel_theta", 1.0, positive);
  acc_lim_x_ = node.declare_parameter<double>(name + ".acc_lim_x", 2.5, positive);
  acc_lim_theta_ = node.declare_parameter<double>(name + ".acc_lim_theta", 3.2, positive);
  decel_lim_x_ = node.declare_parameter<double>(name + ".decel_lim_x", -2.5,
                                                [](double value) { return value < 0.0; });
  decel_lim_theta_ = node.declare_parameter<double>(name + ".decel_lim_theta", -3.2,
                                                    [](double value) { return value < 0.0; });
  vx_samples_ = node.declare_parameter<std::int64_t>(name + ".vx_samples", 6,
                                                     [](std::int64_t value) { return value > 0; });
  vtheta_samples_ = node.declare_parameter<std::int64_t>(name + ".vtheta_samples", 15,
                                                         [](std::int64_t value) { return value > 0; });
  sim_time_ = node.declare_parameter<double>(name + ".sim_time", 1.7, positive);
  prune_distance_ = node.declare_parameter<double>(name + ".prune_distance", 1.0, positive);
  xy_goal_tolerance_ = node.declare_parameter<double>(name + ".xy_goal_tolerance", 0.2, positive);
  rotate_to_goal_scale_ = node.declare_parameter<double>(name + ".RotateToGoal.scale", 32.0, non_negative);
  base_obstacle_scale_ = node.declare_parameter<double>(name + ".BaseObstacle.scale", 0.02, non_negative);
  goal_align_scale_ = node.declare_parameter<double>(name + ".GoalAlign.scale", 24.0, non_negative);
  path_align_scale_ = node.declare_parameter<double>(name + ".PathAlign.scale", 32.0, non_negative);
  path_dist_scale_ = node.declare_parameter<double>(name + ".PathDist.scale", 32.0, non_negative);
  goal_dist_scale_ = node.declare_parameter<double>(name + ".GoalDist.scale", 24.0, non_negative);
  path_align_forward_ = node.declare_parameter<double>(name + ".PathAlign.forward_point_distance", 0.325,
                                                       non_negative);
  goal_align_forward_ = node.declare_parameter<double>(name + ".GoalAlign.forward_point_distance", 0.325,
                                                       non_negative);
}

std::vector<Pose2> DWBLocalPlanner::pruned_path(const Pose2& pose,
                                                 const std::vector<Pose2>& path) const {
  const std::size_t start = closest_index(pose, path);
  std::vector<Pose2> local;
  for (std::size_t index = start; index < path.size(); ++index) {
    if (std::hypot(path[index].x - pose.x, path[index].y - pose.y) <= prune_distance_.value()) {
      local.push_back(path[index]);
    }
  }
  if (local.empty()) {
    local.push_back(path[start]);
  }
  return local;
}

Velocity DWBLocalPlanner::compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                                    const std::vector<Pose2>& path,
                                                    const Costmap& costmap, double dt) {
  const Pose2 goal = path.back();
  if (std::hypot(goal.x - pose.x, goal.y - pose.y) < xy_goal_tolerance_.value()) {
    const double angle = wrap_angle(goal.yaw - pose.yaw);
    const double w = std::copysign(std::min({max_vel_theta_.value(),
                                             std::abs(velocity.angular) + acc_lim_theta_.value() * dt,
                                             std::abs(angle) / std::max(dt, 1e-6)}),
                                   angle);
    local_plan_.clear();
    return Velocity{0.0, w};
  }
  const std::vector<Pose2> local = pruned_path(pose, path);
  const Pose2 local_goal = local.back();
  const double near_end = std::hypot(goal.x - pose.x, goal.y - pose.y);
  const double resolution = std::max(costmap.resolution, 1e-6);
  const double v0 = velocity.linear;
  const double w0 = velocity.angular;
  const std::vector<double> vs =
      samples(std::max(min_vel_x_.value(), v0 + decel_lim_x_.value() * dt),
              std::min(max_vel_x_.value(), v0 + acc_lim_x_.value() * dt), vx_samples_.value());
  const double limit = max_vel_theta_.value();
  const std::vector<double> ws =
      samples(std::max(-limit, w0 + decel_lim_theta_.value() * dt),
              std::min(limit, w0 + acc_lim_theta_.value() * dt), vtheta_samples_.value());
  std::optional<std::pair<double, Velocity>> best;
  std::vector<Pose2> best_trajectory;
  constexpr double kStep = 0.1;
  for (const double v : vs) {
    for (const double w : ws) {
      const std::vector<Pose2> trajectory = simulate(pose, v, w, sim_time_.value(), kStep);
      double obstacle = 0.0;
      bool collision = false;
      for (const auto& point : trajectory) {
        if (std::hypot(point.x - pose.x, point.y - pose.y) > 0.1 &&
            blocked(costmap, point.x, point.y)) {
          collision = true;
          break;
        }
        obstacle += std::min<double>(costmap.cost_at(point.x, point.y), INSCRIBED_INFLATED_OBSTACLE - 1);
      }
      if (collision || trajectory.empty()) {
        continue;
      }
      const Pose2& end = trajectory.back();
      double score = base_obstacle_scale_.value() * obstacle;
      score += path_dist_scale_.value() * distance_to_path(end.x, end.y, local) / resolution;
      score += goal_dist_scale_.value() * std::hypot(end.x - local_goal.x, end.y - local_goal.y) / resolution;
      if (near_end > path_align_forward_.value()) {
        const double ahead_x = end.x + path_align_forward_.value() * std::cos(end.yaw);
        const double ahead_y = end.y + path_align_forward_.value() * std::sin(end.yaw);
        score += path_align_scale_.value() * distance_to_path(ahead_x, ahead_y, local) / resolution;
      }
      if (near_end > goal_align_forward_.value()) {
        const double ahead_x = end.x + goal_align_forward_.value() * std::cos(end.yaw);
        const double ahead_y = end.y + goal_align_forward_.value() * std::sin(end.yaw);
        score += goal_align_scale_.value() *
                 std::hypot(ahead_x - local_goal.x, ahead_y - local_goal.y) / resolution;
      }
      if (!best || score < best->first) {
        best = std::make_pair(score, Velocity{v, w});
        best_trajectory = trajectory;
      }
    }
  }
  if (!best) {
    throw ControllerError(msgs::nav2::FollowPathResult::NO_VALID_CONTROL, "모든 자취가 벽에 닿는다");
  }
  local_plan_ = std::move(best_trajectory);
  return best->second;
}

}  // namespace krosc::nodes::nav2
