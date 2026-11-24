#include "krosc/nodes/nav2/mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace krosc::nodes::nav2 {

namespace {

constexpr double kCollisionCost = 1'000'000.0;

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

}  // namespace

// ---------------------------------------------------------------- MPPI

void MPPIController::configure(Node& node, const std::string& name) {
  NavControllerPlugin::configure(node, name);
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

}  // namespace krosc::nodes::nav2
