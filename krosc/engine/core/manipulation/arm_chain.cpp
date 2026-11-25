#include "krosc/manipulation/arm_chain.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <set>

#include "krosc/tf/math.hpp"

namespace krosc::manipulation {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kPitchWeight = 0.20;  // 1 rad 자세오차를 0.20 m 위치오차와 같은 무게로 본다.

msgs::Transform make_transform(const std::array<double, 3>& xyz, const std::array<double, 3>& rpy) {
  return {{xyz[0], xyz[1], xyz[2]}, tfmath::from_rpy(rpy[0], rpy[1], rpy[2])};
}

double clamp_joint(double value, double lower, double upper) {
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::clamp(value, lower, upper);
}

std::array<double, 4> clamp_joints(const std::array<double, 4>& q, const std::array<double, 4>& lower,
                                   const std::array<double, 4>& upper) {
  std::array<double, 4> out{};
  for (std::size_t i = 0; i < q.size(); ++i) {
    out[i] = clamp_joint(q[i], lower[i], upper[i]);
  }
  return out;
}

bool solve4(std::array<std::array<double, 4>, 4> a, std::array<double, 4> b, std::array<double, 4>& x) {
  for (std::size_t col = 0; col < 4; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < 4; ++row) {
      if (std::abs(a[row][col]) > std::abs(a[pivot][col])) {
        pivot = row;
      }
    }
    if (std::abs(a[pivot][col]) < 1e-14) {
      return false;
    }
    if (pivot != col) {
      std::swap(a[pivot], a[col]);
      std::swap(b[pivot], b[col]);
    }
    for (std::size_t row = col + 1; row < 4; ++row) {
      const double factor = a[row][col] / a[col][col];
      if (factor == 0.0) {
        continue;
      }
      for (std::size_t k = col; k < 4; ++k) {
        a[row][k] -= factor * a[col][k];
      }
      b[row] -= factor * b[col];
    }
  }
  for (int row = 3; row >= 0; --row) {
    double value = b[static_cast<std::size_t>(row)];
    for (std::size_t col = static_cast<std::size_t>(row) + 1; col < 4; ++col) {
      value -= a[static_cast<std::size_t>(row)][col] * x[col];
    }
    x[static_cast<std::size_t>(row)] = value / a[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)];
  }
  return true;
}

struct IkEvaluation {
  msgs::Vector3 position{};
  double pitch = 0.0;
  double yaw = 0.0;
};

double weighted_cost(const IkEvaluation& current, const msgs::Vector3& target, double desired_pitch) {
  const double dx = target.x - current.position.x;
  const double dy = target.y - current.position.y;
  const double dz = target.z - current.position.z;
  const double dp = (desired_pitch - current.pitch) * kPitchWeight;
  return dx * dx + dy * dy + dz * dz + dp * dp;
}

double joint_distance(const std::array<double, 4>& a, const std::array<double, 4>& b) {
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    sum += (a[i] - b[i]) * (a[i] - b[i]);
  }
  return std::sqrt(sum);
}

// s(t) trapezoid. 거리 1을 이동한다. 반환값은 0..1.
double trapezoid_progress(double t, double duration, double distance, double speed, double accel) {
  if (duration <= 0.0 || distance <= 0.0) {
    return 1.0;
  }
  t = std::clamp(t, 0.0, duration);
  const double t_accel = speed / accel;
  if (2.0 * t_accel >= duration) {
    // 삼각 프로파일
    const double half = duration / 2.0;
    const double peak_speed = distance / half;
    const double a = peak_speed / half;
    if (t <= half) {
      return std::min(1.0, 0.5 * a * t * t / distance);
    }
    const double remaining = duration - t;
    return std::min(1.0, 1.0 - 0.5 * a * remaining * remaining / distance);
  }
  const double d_accel = 0.5 * speed * t_accel;
  const double d_decel = d_accel;
  const double d_cruise = std::max(0.0, distance - d_accel - d_decel);
  const double t_cruise = d_cruise / speed;
  double travelled = 0.0;
  if (t <= t_accel) {
    travelled = 0.5 * accel * t * t;
  } else if (t <= t_accel + t_cruise) {
    travelled = d_accel + speed * (t - t_accel);
  } else {
    const double td = t - t_accel - t_cruise;
    travelled = d_accel + d_cruise + speed * td - 0.5 * accel * td * td;
  }
  return std::clamp(travelled / distance, 0.0, 1.0);
}

double profile_duration(double distance, double speed, double accel) {
  if (distance <= 0.0) {
    return 0.0;
  }
  const double v = std::max(1e-6, speed);
  const double a = std::max(1e-6, accel);
  if (distance * a <= v * v) {
    return 2.0 * std::sqrt(distance / a);
  }
  return distance / v + v / a;
}

std::array<double, 4> interpolate_joints(const std::array<double, 4>& a, const std::array<double, 4>& b,
                                         double ratio) {
  std::array<double, 4> out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = a[i] + (b[i] - a[i]) * ratio;
  }
  return out;
}

void fill_tcp_path(const ArmChain& chain, Trajectory& trajectory) {
  trajectory.tcp_path.clear();
  trajectory.tcp_path.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points) {
    trajectory.tcp_path.push_back(chain.tcp(point.position));
  }
}

double max_joint_velocity(const std::vector<TrajectoryPoint>& points) {
  double maximum = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dt = points[i].time - points[i - 1].time;
    if (dt <= 1e-9) {
      continue;
    }
    for (std::size_t j = 0; j < 4; ++j) {
      maximum = std::max(maximum, std::abs(points[i].position[j] - points[i - 1].position[j]) / dt);
    }
  }
  return maximum;
}

double max_joint_acceleration(const std::vector<TrajectoryPoint>& points) {
  double maximum = 0.0;
  for (std::size_t i = 2; i < points.size(); ++i) {
    const double dt1 = points[i - 1].time - points[i - 2].time;
    const double dt2 = points[i].time - points[i - 1].time;
    const double dt = 0.5 * (dt1 + dt2);
    if (dt <= 1e-9) {
      continue;
    }
    for (std::size_t j = 0; j < 4; ++j) {
      const double v1 = (points[i - 1].position[j] - points[i - 2].position[j]) / dt1;
      const double v2 = (points[i].position[j] - points[i - 1].position[j]) / dt2;
      maximum = std::max(maximum, std::abs(v2 - v1) / dt);
    }
  }
  return maximum;
}

bool plan_joint(const ArmChain& chain, const std::array<double, 4>& start, const std::array<double, 4>& goal,
                const PlanOptions& options, Trajectory& trajectory, std::string& error) {
  double max_delta = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    max_delta = std::max(max_delta, std::abs(goal[i] - start[i]));
  }
  const double sample_time = std::max(0.01, options.sample_time);
  trajectory.cartesian = false;
  trajectory.fallback = false;
  if (max_delta < 1e-6) {
    trajectory.points.clear();
    trajectory.points.push_back(TrajectoryPoint{start, 0.0});
    trajectory.duration = 0.0;
    fill_tcp_path(chain, trajectory);
    return true;
  }
  double duration = profile_duration(max_delta, options.max_joint_speed, options.max_joint_accel);
  duration = std::max(duration, sample_time);
  const int steps = std::max(1, static_cast<int>(std::ceil(duration / sample_time)));
  trajectory.points.clear();
  trajectory.points.reserve(static_cast<std::size_t>(steps) + 1);
  for (int step = 0; step <= steps; ++step) {
    const double time = duration * static_cast<double>(step) / static_cast<double>(steps);
    const double ratio = trapezoid_progress(time, duration, max_delta, options.max_joint_speed,
                                            options.max_joint_accel);
    trajectory.points.push_back(TrajectoryPoint{interpolate_joints(start, goal, ratio), time});
  }
  trajectory.points.back().position = goal;
  trajectory.points.back().time = duration;
  trajectory.duration = duration;
  fill_tcp_path(chain, trajectory);
  if (max_joint_velocity(trajectory.points) > options.max_joint_speed * 1.08) {
    error = "joint-space 궤적 속도 한계를 맞추지 못했다";
    return false;
  }
  return true;
}

bool plan_cartesian(const ArmChain& chain, const std::array<double, 4>& start, const std::array<double, 4>& goal,
                    const msgs::Vector3& start_tcp, const msgs::Vector3& goal_tcp, double start_pitch,
                    double goal_pitch, const PlanOptions& options, Trajectory& trajectory, std::string& error) {
  const double dx = goal_tcp.x - start_tcp.x;
  const double dy = goal_tcp.y - start_tcp.y;
  const double dz = goal_tcp.z - start_tcp.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double pitch_delta = std::abs(goal_pitch - start_pitch);
  const double sample_time = std::max(0.01, options.sample_time);
  if (distance < 1e-6 && pitch_delta < 1e-6) {
    return plan_joint(chain, start, goal, options, trajectory, error);
  }
  const double effective_distance = distance + 0.15 * pitch_delta;
  double speed = std::max(0.005, options.cartesian_speed);
  double accel = std::max(0.01, options.cartesian_accel);
  trajectory.points.clear();
  trajectory.cartesian = true;
  trajectory.fallback = false;
  trajectory.note.clear();
  std::array<double, 4> seed = start;
  for (int retry = 0; retry < 6; ++retry) {
    double duration = std::max(profile_duration(effective_distance, speed, accel), sample_time);
    const int steps = std::max(1, static_cast<int>(std::ceil(duration / sample_time)));
    constexpr int kMaxCartesianSteps = 4000;  // 0.05s 샘플 기준 200초. 넘으면 joint-space fallback.
    if (steps > kMaxCartesianSteps) {
      error = "Cartesian 경로가 너무 길다 (" + std::to_string(steps) + " steps)";
      trajectory.note = error;
      trajectory.points.clear();
      return false;
    }
    std::vector<TrajectoryPoint> points;
    points.reserve(static_cast<std::size_t>(steps) + 1);
    seed = start;
    for (int step = 0; step <= steps; ++step) {
      const double time = duration * static_cast<double>(step) / static_cast<double>(steps);
      const double ratio = trapezoid_progress(time, duration, effective_distance, speed, accel);
      msgs::Vector3 target{start_tcp.x + ratio * dx, start_tcp.y + ratio * dy, start_tcp.z + ratio * dz};
      const double pitch = start_pitch + ratio * (goal_pitch - start_pitch);
      std::array<double, 4> solution{};
      std::string ik_error;
      // 경로 샘플은 이전 해(seed) 근처의 해면 충분하다. fast=false로 두면
      // 도달 불가능한 경로에서 모든 seed×iteration을 매 스텝 반복해 수 분씩
      // 블록될 수 있다. fast=true로 실패를 빨리 감지하고 joint-space로 대체한다.
      if (!chain.ik(target, pitch, seed, solution, ik_error, true)) {
        std::string message = "Cartesian 경로 IK 실패 (t=" + std::to_string(time) + "s): " + ik_error;
        error = message;
        trajectory.note = message;
        trajectory.points.clear();
        return false;
      }
      seed = solution;
      if (step == steps) {
        solution = goal;
      }
      points.push_back(TrajectoryPoint{solution, time});
    }
    points.back().position = goal;
    points.back().time = duration;
    const double velocity = max_joint_velocity(points);
    const double acceleration = max_joint_acceleration(points);
    if (velocity > options.max_joint_speed * 1.05 || acceleration > options.max_joint_accel * 1.25) {
      const double scale = std::max(velocity / (options.max_joint_speed * 1.02),
                                    std::sqrt(acceleration / (options.max_joint_accel * 1.15)));
      // 시간만 늘리면 프로파일 속도는 그대로다. 속도/가속도 자체를 낮춰 다시 샘플링한다.
      const double factor = std::max(1.05, scale);
      speed /= factor;
      accel /= factor * factor;
      continue;
    }
    trajectory.points = std::move(points);
    trajectory.duration = duration;
    fill_tcp_path(chain, trajectory);
    return true;
  }
  error = "Cartesian 궤적의 관절 속도/가속도 한계를 맞추지 못했다";
  trajectory.points.clear();
  return false;
}

}  // namespace

std::optional<ArmChain> ArmChain::from_urdf(const UrdfRobot& robot, const std::string& base_link,
                                            const std::string& tip_link,
                                            const std::vector<std::string>& joint_names,
                                            std::string& error) {
  if (joint_names.size() != 4) {
    error = "팔 관절은 정확히 4개여야 한다";
    return std::nullopt;
  }
  if (robot.find_link(base_link) == nullptr) {
    error = "base_link를 URDF에서 찾을 수 없다: " + base_link;
    return std::nullopt;
  }
  if (robot.find_link(tip_link) == nullptr) {
    error = "tip_link를 URDF에서 찾을 수 없다: " + tip_link;
    return std::nullopt;
  }
  std::map<std::string, const UrdfJoint*> parent_joint;
  for (const auto& joint : robot.joints) {
    parent_joint[joint.child] = &joint;
  }
  std::vector<const UrdfJoint*> reversed;
  std::set<std::string> visited;
  std::string current = tip_link;
  while (current != base_link) {
    const auto found = parent_joint.find(current);
    if (found == parent_joint.end()) {
      error = tip_link + "에서 " + base_link + "로 가는 관절 체인이 없다";
      return std::nullopt;
    }
    if (!visited.insert(current).second) {
      error = "URDF 관절 체인에 순환이 있다";
      return std::nullopt;
    }
    reversed.push_back(found->second);
    current = found->second->parent;
  }
  if (reversed.empty()) {
    error = "base_link와 tip_link가 같다";
    return std::nullopt;
  }
  std::reverse(reversed.begin(), reversed.end());

  ArmChain chain;
  chain.base_link = base_link;
  chain.tip_link = tip_link;
  chain.joint_names = joint_names;
  std::array<bool, 4> seen{{false, false, false, false}};
  std::array<bool, 4> limit_seen{{false, false, false, false}};
  bool before_first_movable = true;
  msgs::Transform running{};
  for (const UrdfJoint* joint : reversed) {
    ArmChainSegment segment;
    segment.joint = joint->name;
    segment.child_link = joint->child;
    segment.type = joint->type;
    segment.origin = make_transform(joint->xyz, joint->rpy);
    segment.axis = {joint->axis[0], joint->axis[1], joint->axis[2]};
    segment.movable = joint->movable();
    if (joint->type == "prismatic") {
      segment.prismatic = true;
    }
    if (segment.movable) {
      const auto found = std::find(joint_names.begin(), joint_names.end(), joint->name);
      if (found == joint_names.end()) {
        error = "체인에 설정에 없는 movable 관절이 있다: " + joint->name;
        return std::nullopt;
      }
      const int index = static_cast<int>(std::distance(joint_names.begin(), found));
      if (seen[static_cast<std::size_t>(index)]) {
        error = "관절이 체인에 두 번 나온다: " + joint->name;
        return std::nullopt;
      }
      seen[static_cast<std::size_t>(index)] = true;
      segment.joint_index = index;
      if (joint->has_limits) {
        chain.lower[static_cast<std::size_t>(index)] = joint->lower;
        chain.upper[static_cast<std::size_t>(index)] = joint->upper;
        limit_seen[static_cast<std::size_t>(index)] = true;
      } else if (joint->type == "continuous") {
        chain.lower[static_cast<std::size_t>(index)] = -kPi;
        chain.upper[static_cast<std::size_t>(index)] = kPi;
      }
      if (before_first_movable) {
        chain.base_to_proximal = running;
        chain.proximal_joint_origin = segment.origin;
        before_first_movable = false;
      }
    } else {
      running = tfmath::compose(running, segment.origin);
    }
    chain.segments.push_back(std::move(segment));
  }
  for (std::size_t i = 0; i < seen.size(); ++i) {
    if (!seen[i]) {
      error = "체인에 관절이 없다: " + joint_names[i];
      return std::nullopt;
    }
    if (!limit_seen[i] && (chain.lower[i] == chain.upper[i])) {
      error = "관절 한계를 읽지 못했다: " + joint_names[i];
      return std::nullopt;
    }
  }
  return chain;
}

msgs::Transform ArmChain::fk(const std::array<double, 4>& q) const {
  msgs::Transform transform{};
  for (const auto& segment : segments) {
    transform = tfmath::compose(transform, segment.origin);
    if (segment.movable) {
      const double value = q[static_cast<std::size_t>(segment.joint_index)];
      if (segment.prismatic) {
        transform = tfmath::compose(
            transform, msgs::Transform{{segment.axis.x * value, segment.axis.y * value, segment.axis.z * value},
                                       {}});
      } else {
        transform = tfmath::compose(
            transform, msgs::Transform{{}, tfmath::from_axis_angle(segment.axis, value)});
      }
    }
  }
  return transform;
}

std::vector<std::pair<std::string, msgs::Transform>> ArmChain::link_transforms(
    const std::array<double, 4>& q) const {
  std::vector<std::pair<std::string, msgs::Transform>> out;
  msgs::Transform transform{};
  for (const auto& segment : segments) {
    transform = tfmath::compose(transform, segment.origin);
    if (segment.movable) {
      const double value = q[static_cast<std::size_t>(segment.joint_index)];
      if (segment.prismatic) {
        transform = tfmath::compose(
            transform, msgs::Transform{{segment.axis.x * value, segment.axis.y * value, segment.axis.z * value},
                                       {}});
      } else {
        transform = tfmath::compose(
            transform, msgs::Transform{{}, tfmath::from_axis_angle(segment.axis, value)});
      }
    }
    out.emplace_back(segment.child_link, transform);
  }
  return out;
}

msgs::Vector3 ArmChain::tcp(const std::array<double, 4>& q) const { return fk(q).translation; }

double ArmChain::pitch_down(const std::array<double, 4>& q) const {
  const msgs::Vector3 axis = tfmath::rotate(fk(q).rotation, {1.0, 0.0, 0.0});
  return -std::asin(std::clamp(axis.z, -1.0, 1.0));
}

double ArmChain::tool_yaw(const std::array<double, 4>& q) const {
  const msgs::Vector3 axis = tfmath::rotate(fk(q).rotation, {1.0, 0.0, 0.0});
  return std::atan2(axis.y, axis.x);
}

bool ArmChain::ik(const msgs::Vector3& target, double desired_pitch, const std::array<double, 4>& seed,
                  std::array<double, 4>& solution, std::string& error, bool fast) const {
  if (!valid()) {
    error = "URDF 체인이 준비되지 않았다";
    return false;
  }
  if (!std::isfinite(target.x) || !std::isfinite(target.y) || !std::isfinite(target.z) ||
      !std::isfinite(desired_pitch)) {
    error = "목표가 유한한 값이 아니다";
    return false;
  }
  desired_pitch = std::clamp(desired_pitch, -1.5, 1.5);

  // q1 초기 추정: link1 평면에서 목표 방향.
  const msgs::Vector3 in_proximal = tfmath::apply(tfmath::inverse(base_to_proximal), target);
  const double q1_guess =
      std::atan2(in_proximal.y, in_proximal.x - proximal_joint_origin.translation.x);

  const auto evaluate = [this](const std::array<double, 4>& q) {
    const msgs::Transform tip = fk(q);
    IkEvaluation result;
    result.position = tip.translation;
    const msgs::Vector3 axis = tfmath::rotate(tip.rotation, {1.0, 0.0, 0.0});
    result.pitch = -std::asin(std::clamp(axis.z, -1.0, 1.0));
    result.yaw = std::atan2(axis.y, axis.x);
    return result;
  };

  std::vector<std::array<double, 4>> seeds;
  seeds.push_back(clamp_joints(seed, lower, upper));
  {
    auto q = seed;
    q[0] = q1_guess;
    seeds.push_back(clamp_joints(q, lower, upper));
  }
  seeds.push_back(clamp_joints({q1_guess, -1.9024, 1.3440, 0.9023}, lower, upper));
  seeds.push_back(clamp_joints({q1_guess, 0.0, 0.0, 0.0}, lower, upper));
  seeds.push_back(clamp_joints({q1_guess, -0.6, 0.8, 0.2}, lower, upper));
  seeds.push_back(clamp_joints({q1_guess, -1.2, 1.2, 0.4}, lower, upper));
  seeds.push_back(clamp_joints({q1_guess, 0.4, -0.4, 0.0}, lower, upper));
  seeds.push_back(clamp_joints({q1_guess, -2.2, 1.8, 1.2}, lower, upper));
  seeds.push_back(clamp_joints({q1_guess + 0.35, -1.2, 1.0, 0.5}, lower, upper));
  seeds.push_back(clamp_joints({q1_guess - 0.35, -1.2, 1.0, 0.5}, lower, upper));

  double best_score = std::numeric_limits<double>::infinity();
  std::array<double, 4> best{};
  double best_position_error = std::numeric_limits<double>::infinity();
  double best_pitch_error = std::numeric_limits<double>::infinity();
  bool found = false;

  for (const auto& start : seeds) {
    std::array<double, 4> q = start;
    IkEvaluation current = evaluate(q);
    double cost = weighted_cost(current, target, desired_pitch);
    for (int iteration = 0; iteration < 260; ++iteration) {
      const double dx = target.x - current.position.x;
      const double dy = target.y - current.position.y;
      const double dz = target.z - current.position.z;
      const double dp = desired_pitch - current.pitch;
      const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (position_error < 3e-4 && std::abs(dp) < 3e-3) {
        break;
      }
      std::array<std::array<double, 4>, 4> jacobian{};
      constexpr double epsilon = 1e-6;
      for (std::size_t joint = 0; joint < 4; ++joint) {
        std::array<double, 4> perturbed = q;
        double delta = epsilon;
        if (perturbed[joint] + delta > upper[joint]) {
          delta = -epsilon;
        }
        perturbed[joint] += delta;
        const IkEvaluation moved = evaluate(perturbed);
        jacobian[0][joint] = (moved.position.x - current.position.x) / delta;
        jacobian[1][joint] = (moved.position.y - current.position.y) / delta;
        jacobian[2][joint] = (moved.position.z - current.position.z) / delta;
        jacobian[3][joint] = (moved.pitch - current.pitch) / delta * kPitchWeight;
      }
      std::array<double, 4> error_vector{dx, dy, dz, dp * kPitchWeight};
      // A = J^T J + lambda^2 I, b = J^T e
      std::array<std::array<double, 4>, 4> a{};
      std::array<double, 4> b{};
      for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t col = 0; col < 4; ++col) {
          double sum = 0.0;
          for (std::size_t k = 0; k < 4; ++k) {
            sum += jacobian[k][row] * jacobian[k][col];
          }
          a[row][col] = sum;
        }
        double sum = 0.0;
        for (std::size_t k = 0; k < 4; ++k) {
          sum += jacobian[k][row] * error_vector[k];
        }
        b[row] = sum;
      }
      const double lambda = 0.02;
      for (std::size_t i = 0; i < 4; ++i) {
        a[i][i] += lambda * lambda;
      }
      std::array<double, 4> delta_q{};
      if (!solve4(a, b, delta_q)) {
        break;
      }
      double max_step = 0.0;
      for (double value : delta_q) {
        max_step = std::max(max_step, std::abs(value));
      }
      if (max_step > 0.35) {
        for (double& value : delta_q) {
          value *= 0.35 / max_step;
        }
      }
      bool accepted = false;
      double scale = 1.0;
      for (int line = 0; line < 6; ++line) {
        std::array<double, 4> candidate = q;
        for (std::size_t i = 0; i < 4; ++i) {
          candidate[i] = clamp_joint(q[i] + scale * delta_q[i], lower[i], upper[i]);
        }
        const IkEvaluation moved = evaluate(candidate);
        const double moved_cost = weighted_cost(moved, target, desired_pitch);
        if (moved_cost < cost) {
          q = candidate;
          current = moved;
          cost = moved_cost;
          accepted = true;
          break;
        }
        scale *= 0.5;
      }
      if (!accepted) {
        break;
      }
    }
    const double dx = target.x - current.position.x;
    const double dy = target.y - current.position.y;
    const double dz = target.z - current.position.z;
    const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double pitch_error = std::abs(desired_pitch - current.pitch);
    const bool converged = position_error < 8e-4 && pitch_error < 8e-3;
    if (converged) {
      // 같은 목표를 만족하는 해가 여러 개면 현재 seed에 가장 가까운 해를 고른다.
      // (팔꿈치 위/아래 해가 바뀌면 궤적에 작은 점프가 생긴다.)
      const double score = joint_distance(q, seed) + position_error * 50.0 + pitch_error * 5.0;
      if (!found || score < best_score) {
        found = true;
        best = q;
        best_score = score;
        best_position_error = position_error;
        best_pitch_error = pitch_error;
      }
      if (fast) {
        break;  // 드래그 preview: 첫 번째로 만족한 해를 바로 쓴다
      }
    } else {
      const double score = 10.0 + position_error / 0.001 + pitch_error / 0.01;
      if (!found && score < best_score) {
        best_score = score;
        best = q;
        best_position_error = position_error;
        best_pitch_error = pitch_error;
      }
    }
  }

  if (!found) {
    char message[160];
    std::snprintf(message, sizeof(message), "IK 도달 불가 (pos err %.3f m, pitch err %.1f°)",
                  best_position_error, best_pitch_error * 180.0 / kPi);
    error = message;
    return false;
  }
  solution = best;
  error.clear();
  (void)best_score;
  return true;
}

std::optional<Trajectory> plan_trajectory(const ArmChain& chain, const std::array<double, 4>& start,
                                          const std::array<double, 4>& goal, const PlanOptions& options,
                                          std::string& error) {
  if (!chain.valid()) {
    error = "URDF 체인이 준비되지 않았다";
    return std::nullopt;
  }
  Trajectory trajectory;
  if (!options.cartesian) {
    if (!plan_joint(chain, start, goal, options, trajectory, error)) {
      return std::nullopt;
    }
    return trajectory;
  }
  const msgs::Vector3 start_tcp = chain.tcp(start);
  const msgs::Vector3 goal_tcp = chain.tcp(goal);
  const double start_pitch = chain.pitch_down(start);
  const double goal_pitch = chain.pitch_down(goal);
  std::string cartesian_error;
  if (plan_cartesian(chain, start, goal, start_tcp, goal_tcp, start_pitch, goal_pitch, options, trajectory,
                     cartesian_error)) {
    return trajectory;
  }
  if (!plan_joint(chain, start, goal, options, trajectory, error)) {
    error = cartesian_error + " / joint-space도 실패: " + error;
    return std::nullopt;
  }
  trajectory.fallback = true;
  trajectory.note = cartesian_error + " → joint-space로 대체";
  return trajectory;
}

}  // namespace krosc::manipulation
