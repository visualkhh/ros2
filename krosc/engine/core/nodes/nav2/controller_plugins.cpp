#include "krosc/nodes/nav2/controller_plugins.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace krosc::nodes::nav2 {

namespace {

constexpr double kCollisionCost = 1'000'000.0;

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

std::vector<Pose2> rollout(const Pose2& pose, const std::vector<Velocity>& controls, double dt) {
  double x = pose.x;
  double y = pose.y;
  double yaw = pose.yaw;
  std::vector<Pose2> points;
  points.reserve(controls.size());
  for (const auto& command : controls) {
    yaw += command.angular * dt;
    x += command.linear * std::cos(yaw) * dt;
    y += command.linear * std::sin(yaw) * dt;
    points.push_back(Pose2{x, y, yaw});
  }
  return points;
}

std::vector<Velocity> smooth(const std::vector<Velocity>& controls, std::int64_t window) {
  if (window <= 1 || controls.size() < 3) {
    return controls;
  }
  const std::size_t half = static_cast<std::size_t>(window / 2);
  std::vector<Velocity> result;
  result.reserve(controls.size());
  for (std::size_t i = 0; i < controls.size(); ++i) {
    const std::size_t lo = i > half ? i - half : 0;
    const std::size_t hi = std::min(controls.size(), i + half + 1);
    double linear = 0.0;
    double angular = 0.0;
    for (std::size_t k = lo; k < hi; ++k) {
      linear += controls[k].linear;
      angular += controls[k].angular;
    }
    const double n = static_cast<double>(hi - lo);
    result.push_back(Velocity{linear / n, angular / n});
  }
  return result;
}

double distance_to_path(double x, double y, const std::vector<Pose2>& path) {
  double best = std::numeric_limits<double>::infinity();
  for (const auto& point : path) {
    best = std::min(best, std::hypot(x - point.x, y - point.y));
  }
  return best;
}

}  // namespace

// ---------------------------------------------------------------- RPP

void RegulatedPurePursuitController::configure(Node& node, const std::string& name) {
  Controller::configure(node, name);
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

// ---------------------------------------------------------------- DWB

void DWBLocalPlanner::configure(Node& node, const std::string& name) {
  Controller::configure(node, name);
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

// ---------------------------------------------------------------- MPPI

void MPPIController::configure(Node& node, const std::string& name) {
  Controller::configure(node, name);
  const auto positive = [](double value) { return value > 0.0; };
  const auto non_negative = [](double value) { return value >= 0.0; };
  batch_size_ = node.declare_parameter<std::int64_t>(name + ".batch_size", 80,
                                                     [](std::int64_t value) { return value > 0; });
  time_steps_ = node.declare_parameter<std::int64_t>(name + ".time_steps", 20,
                                                     [](std::int64_t value) { return value > 0; });
  model_dt_ = node.declare_parameter<double>(name + ".model_dt", 0.1, positive);
  vx_std_ = node.declare_parameter<double>(name + ".vx_std", 0.2, non_negative);
  wz_std_ = node.declare_parameter<double>(name + ".wz_std", 0.2, non_negative);
  vx_max_ = node.declare_parameter<double>(name + ".vx_max", 0.22, positive);
  vx_min_ = node.declare_parameter<double>(name + ".vx_min", -0.1,
                                           [](double value) { return value <= 0.0; });
  wz_max_ = node.declare_parameter<double>(name + ".wz_max", 1.5, positive);
  temperature_ = node.declare_parameter<double>(name + ".temperature", 0.3, positive);
  smoothing_window_ = node.declare_parameter<std::int64_t>(name + ".smoothing_window", 7,
                                                           [](std::int64_t value) { return value >= 1; });
  seed_ = node.declare_parameter<std::int64_t>(name + ".seed", 0);
  cost_critic_weight_ = node.declare_parameter<double>(name + ".CostCritic.cost_weight", 3.81, non_negative);
  goal_critic_weight_ = node.declare_parameter<double>(name + ".GoalCritic.cost_weight", 5.0, non_negative);
  goal_angle_critic_weight_ =
      node.declare_parameter<double>(name + ".GoalAngleCritic.cost_weight", 3.0, non_negative);
  path_align_critic_weight_ =
      node.declare_parameter<double>(name + ".PathAlignCritic.cost_weight", 14.0, non_negative);
  path_follow_critic_weight_ =
      node.declare_parameter<double>(name + ".PathFollowCritic.cost_weight", 5.0, non_negative);
  path_angle_critic_weight_ =
      node.declare_parameter<double>(name + ".PathAngleCritic.cost_weight", 2.0, non_negative);
  prefer_forward_critic_weight_ =
      node.declare_parameter<double>(name + ".PreferForwardCritic.cost_weight", 5.0, non_negative);
  goal_threshold_ = node.declare_parameter<double>(name + ".GoalCritic.threshold_to_consider", 1.0, positive);
  goal_angle_threshold_ =
      node.declare_parameter<double>(name + ".GoalAngleCritic.threshold_to_consider", 0.2, positive);
  path_follow_threshold_ =
      node.declare_parameter<double>(name + ".PathFollowCritic.threshold_to_consider", 1.0, positive);
  path_follow_offset_ = node.declare_parameter<std::int64_t>(
      name + ".PathFollowCritic.offset_from_furthest", 6, [](std::int64_t value) { return value > 0; });
  path_angle_offset_ = node.declare_parameter<std::int64_t>(
      name + ".PathAngleCritic.offset_from_furthest", 4, [](std::int64_t value) { return value > 0; });
  max_angle_to_furthest_ =
      node.declare_parameter<double>(name + ".PathAngleCritic.max_angle_to_furthest", 0.785,
                                     positive);
  rng_.seed(static_cast<std::mt19937::result_type>(seed_.value()));
}

double MPPIController::score(const std::vector<Pose2>& trajectory,
                            const std::vector<Velocity>& controls, const Costmap& costmap,
                            const std::vector<Pose2>& local, const Pose2& goal, double to_goal,
                            const Pose2& follow, const Pose2& ahead, const Pose2& pose,
                            double model_dt) const {
  const double n = static_cast<double>(std::max<std::size_t>(1, trajectory.size()));
  double result = 0.0;
  double cost_sum = 0.0;
  for (const auto& point : trajectory) {
    const double cost = costmap.cost_at(point.x, point.y);
    if (cost >= INSCRIBED_INFLATED_OBSTACLE && std::hypot(point.x - pose.x, point.y - pose.y) > 0.1) {
      return kCollisionCost;
    }
    cost_sum += std::min<double>(cost, INSCRIBED_INFLATED_OBSTACLE - 1) / 252.0;
  }
  result += cost_critic_weight_.value() * cost_sum / n;
  const Pose2& end = trajectory.back();
  if (to_goal < goal_threshold_.value()) {
    double sum = 0.0;
    for (const auto& point : trajectory) {
      sum += std::hypot(point.x - goal.x, point.y - goal.y);
    }
    result += goal_critic_weight_.value() * sum / n;
  }
  if (to_goal < goal_angle_threshold_.value()) {
    double sum = 0.0;
    for (const auto& point : trajectory) {
      sum += std::abs(wrap_angle(point.yaw - goal.yaw));
    }
    result += goal_angle_critic_weight_.value() * sum / n;
  }
  if (to_goal >= goal_angle_threshold_.value()) {
    double sum = 0.0;
    std::size_t count = 0;
    for (std::size_t index = 2; index < trajectory.size(); index += 3) {
      sum += distance_to_path(trajectory[index].x, trajectory[index].y, local);
      ++count;
    }
    result += path_align_critic_weight_.value() * sum / static_cast<double>(std::max<std::size_t>(1, count));
  }
  if (to_goal >= path_follow_threshold_.value()) {
    result += path_follow_critic_weight_.value() * std::hypot(end.x - follow.x, end.y - follow.y);
  }
  const double angle_to_ahead = std::atan2(ahead.y - pose.y, ahead.x - pose.x);
  if (to_goal >= goal_angle_threshold_.value() &&
      std::abs(wrap_angle(angle_to_ahead - pose.yaw)) > max_angle_to_furthest_.value()) {
    double sum = 0.0;
    for (const auto& point : trajectory) {
      sum += std::abs(wrap_angle(std::atan2(ahead.y - point.y, ahead.x - point.x) - point.yaw));
    }
    result += path_angle_critic_weight_.value() * sum / n;
  }
  double backward = 0.0;
  for (const auto& command : controls) {
    backward += std::max(0.0, -command.linear) * model_dt;
  }
  result += prefer_forward_critic_weight_.value() * backward;
  return result;
}

Velocity MPPIController::compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                                    const std::vector<Pose2>& path,
                                                    const Costmap& costmap, double dt) {
  (void)dt;
  const std::size_t steps = static_cast<std::size_t>(time_steps_.value());
  const double model_dt = model_dt_.value();
  if (control_.size() != steps) {
    control_.assign(steps, velocity);
  }
  const double vx_min = vx_min_.value();
  const double vx_max = vx_max_.value();
  const double wz_max = wz_max_.value();
  const std::size_t start = closest_index(pose, path);
  const std::size_t reach =
      static_cast<std::size_t>(static_cast<int>(vx_max * static_cast<double>(steps) * model_dt /
                                                std::max(costmap.resolution, 1e-6)) *
                                   2 +
                               10);
  std::vector<Pose2> local;
  for (std::size_t index = start; index < path.size() && index < start + reach; index += 2) {
    local.push_back(path[index]);
  }
  if (local.empty()) {
    local.push_back(path.back());
  }
  const Pose2 goal = path.back();
  const double to_goal = std::hypot(goal.x - pose.x, goal.y - pose.y);
  const Pose2 follow =
      path[std::min(path.size() - 1, start + static_cast<std::size_t>(path_follow_offset_.value()))];
  const Pose2 ahead =
      path[std::min(path.size() - 1, start + static_cast<std::size_t>(path_angle_offset_.value()))];

  struct Candidate {
    double score = 0.0;
    std::vector<Velocity> controls;
    std::vector<Pose2> trajectory;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<std::size_t>(batch_size_.value()));
  std::normal_distribution<double> linear_noise(0.0, vx_std_.value());
  std::normal_distribution<double> angular_noise(0.0, wz_std_.value());
  for (std::int64_t k = 0; k < batch_size_.value(); ++k) {
    std::vector<Velocity> noisy;
    if (k == 0) {
      noisy = control_;
    } else {
      const double dv = linear_noise(rng_);
      const double dw = angular_noise(rng_);
      noisy.reserve(control_.size());
      for (const auto& command : control_) {
        noisy.push_back(Velocity{std::min(vx_max, std::max(vx_min, command.linear + dv)),
                                 std::min(wz_max, std::max(-wz_max, command.angular + dw))});
      }
    }
    std::vector<Pose2> trajectory = rollout(pose, noisy, model_dt);
    Candidate candidate;
    candidate.score = score(trajectory, noisy, costmap, local, goal, to_goal, follow, ahead, pose,
                            model_dt);
    candidate.controls = std::move(noisy);
    candidate.trajectory = std::move(trajectory);
    candidates.push_back(std::move(candidate));
  }
  double lowest = std::numeric_limits<double>::infinity();
  for (const auto& candidate : candidates) {
    lowest = std::min(lowest, candidate.score);
  }
  if (lowest >= kCollisionCost) {
    control_.clear();
    throw ControllerError(msgs::nav2::FollowPathResult::NO_VALID_CONTROL, "모든 후보가 벽에 닿는다");
  }
  const double temperature = temperature_.value();
  std::vector<double> weights;
  weights.reserve(candidates.size());
  double total = 0.0;
  for (const auto& candidate : candidates) {
    const double weight = std::exp(-(candidate.score - lowest) / temperature);
    weights.push_back(weight);
    total += weight;
  }
  std::vector<Velocity> averaged;
  averaged.reserve(steps);
  for (std::size_t step = 0; step < steps; ++step) {
    double linear = 0.0;
    double angular = 0.0;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      linear += weights[index] * candidates[index].controls[step].linear;
      angular += weights[index] * candidates[index].controls[step].angular;
    }
    averaged.push_back(Velocity{linear / total, angular / total});
  }
  control_ = smooth(averaged, smoothing_window_.value());
  local_plan_ = rollout(pose, control_, model_dt);
  const Velocity command = control_.front();
  control_.erase(control_.begin());
  control_.push_back(control_.back());
  return command;
}

ControllerRegistry builtin_controllers() {
  ControllerRegistry registry;
  registry.add("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
               [] { return std::make_unique<RegulatedPurePursuitController>(); });
  registry.add("dwb_core::DWBLocalPlanner", [] { return std::make_unique<DWBLocalPlanner>(); });
  registry.add("nav2_mppi_controller::MPPIController", [] { return std::make_unique<MPPIController>(); });
  return registry;
}

}  // namespace krosc::nodes::nav2
