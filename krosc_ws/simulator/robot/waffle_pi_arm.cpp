#include "robot/waffle_pi_arm.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include "krosc/tf/math.hpp"

#include "robot/urdf_robot_visualizer.hpp"

namespace krosc_ws::sim {

namespace msgs = krosc::msgs;
namespace tfmath = krosc::tfmath;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kMaxSubstep = 0.05;

double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

std::pair<double, double> wheel_speeds(const msgs::Twist& command, double wheel_radius,
                                       double separation) {
  const double half = separation / 2.0;
  return {(command.linear.x - command.angular.z * half) / wheel_radius,
          (command.linear.x + command.angular.z * half) / wheel_radius};
}

}  // namespace

WafflePiArm::WafflePiArm(bool noisy, std::optional<std::uint32_t> seed,
                         std::string urdf_path)
    : noisy_(noisy), rng_(seed ? *seed : std::random_device{}()) {
  radius = kBodyRadius;
  height = kBodyHeight;
  mass = 1.8;
  friction = 0.3;
  movable = false;

  if (urdf_path.empty()) {
    return;
  }
  std::ifstream stream(urdf_path);
  if (!stream) {
    std::fprintf(stderr, "[waffle_pi_arm] URDF를 읽지 못했다: %s\n", urdf_path.c_str());
    return;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  try {
    visualizer_ = std::make_unique<UrdfRobotVisualizer>(buffer.str());
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[waffle_pi_arm] URDF visual 파싱 실패: %s\n", error.what());
  }
}

WafflePiArm::~WafflePiArm() = default;

void WafflePiArm::set_command(const msgs::Twist& command) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_ = command;
}

msgs::Twist WafflePiArm::command() const {
  std::lock_guard<std::mutex> lock(command_mutex_);
  return command_;
}

void WafflePiArm::tick(double time, double dt) {
  if (dt <= 0.0) {
    return;
  }
  msgs::Twist command;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command = command_;
  }
  const auto [left_target, right_target] = wheel_speeds(command, kWheelRadius, kWheelSeparation);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    left_velocity_ = left_target;
    right_velocity_ = right_target;
    left_angle_ += left_velocity_ * dt;
    right_angle_ += right_velocity_ * dt;
    left_position_ = left_angle_;
    right_position_ = right_angle_;
  }

  // 정지 중에는 바퀴 슬립 잡음을 넣지 않는다. 실제 바퀴도 구동하지 않으면
  // 제자리에서 랜덤하게 굴러가지 않는다.
  const double contact_left =
      noisy_ && std::abs(left_velocity_) > 1e-6 ? left_slip_.apply(left_velocity_, rng_)
                                                : left_velocity_;
  const double contact_right =
      noisy_ && std::abs(right_velocity_) > 1e-6 ? right_slip_.apply(right_velocity_, rng_)
                                                 : right_velocity_;
  const double linear = kWheelRadius * (contact_left + contact_right) / 2.0;
  const double angular = kWheelRadius * (contact_right - contact_left) / kWheelSeparation;
  apply_motion(dt, linear, angular);

  imu_accum_ += dt;
  if (imu_accum_ + 1e-12 >= kImuPeriod) {
    const double imu_dt = imu_accum_;
    imu_accum_ = 0.0;
    sample_imu(time, imu_dt);
  }
  scan_accum_ += dt;
  if (scan_accum_ + 1e-12 >= kScanPeriod) {
    scan_accum_ = 0.0;
    sample_scan(time);
  }

  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    if (manual_) {
      step_towards(manual_position_, manual_target_, dt);
    } else {
      controller_position_ = controller_target_;
    }
  }
}

void WafflePiArm::apply_motion(double dt, double linear, double angular) {
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(linear) * dt / kMaxSubstep)));
  const double h = dt / static_cast<double>(steps);
  for (int i = 0; i < steps; ++i) {
    const double middle = pose.theta + angular * h / 2.0;
    double x = pose.x + linear * std::cos(middle) * h;
    double y = pose.y + linear * std::sin(middle) * h;
    if (world != nullptr) {
      const auto [px, py] = world->push_out(x, y, radius);
      x = px;
      y = py;
    }
    pose.x = x;
    pose.y = y;
    pose.theta = wrap(pose.theta + angular * h);
  }
  last_linear_velocity_ = linear;
  last_angular_velocity_ = angular;
}

msgs::Pose2D WafflePiArm::scan_pose() const {
  return {pose.x + kLidarMountX * std::cos(pose.theta),
          pose.y + kLidarMountX * std::sin(pose.theta), pose.theta};
}

std::map<std::string, double> WafflePiArm::joint_positions() const {
  std::map<std::string, double> result;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    result["wheel_left_joint"] = left_position_;
    result["wheel_right_joint"] = right_position_;
  }
  {
    std::lock_guard<std::mutex> lock(arm_mutex_);
    result["joint1"] = manual_ ? manual_position_[0] : controller_position_[0];
    result["joint2"] = manual_ ? manual_position_[1] : controller_position_[1];
    result["joint3"] = manual_ ? manual_position_[2] : controller_position_[2];
    result["joint4"] = manual_ ? manual_position_[3] : controller_position_[3];
    result["gripper_left_joint"] = manual_ ? manual_position_[4] : controller_position_[4];
    result["gripper_right_joint"] = result["gripper_left_joint"];
  }
  return result;
}

double WafflePiArm::left_position() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return left_position_;
}

double WafflePiArm::right_position() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return right_position_;
}

double WafflePiArm::left_velocity() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return left_velocity_;
}

double WafflePiArm::right_velocity() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return right_velocity_;
}

void WafflePiArm::set_scan_callback(
    std::function<void(const krosc::msgs::LaserScan&)> callback) {
  scan_callback_ = std::move(callback);
}

void WafflePiArm::set_imu_callback(
    std::function<void(const krosc::msgs::Imu&)> callback) {
  imu_callback_ = std::move(callback);
}

void WafflePiArm::sample_scan(double time) {
  const double inc = kTwoPi / kScanBeams;
  const double sensor_x = pose.x + kLidarMountX * std::cos(pose.theta);
  const double sensor_y = pose.y + kLidarMountX * std::sin(pose.theta);
  std::vector<double> angles(static_cast<std::size_t>(kScanBeams));
  for (int i = 0; i < kScanBeams; ++i) {
    angles[static_cast<std::size_t>(i)] = pose.theta + static_cast<double>(i) * inc;
  }
  std::vector<float> hits;
  if (world != nullptr) {
    world->ray_distances(sensor_x, sensor_y, angles, kLidarMountHeight, hits);
  } else {
    hits.assign(static_cast<std::size_t>(kScanBeams), std::numeric_limits<float>::infinity());
  }
  for (auto& range : hits) {
    if (!(static_cast<double>(range) >= kScanRangeMin &&
          static_cast<double>(range) <= kScanRangeMax)) {
      range = std::numeric_limits<float>::infinity();
    }
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    truth_scan_ranges_ = hits;
    truth_scan_pose_ = {sensor_x, sensor_y, pose.theta};
  }
  msgs::LaserScan scan;
  scan.header.stamp = time;
  scan.header.frame_id = "base_scan";
  scan.angle_min = 0.0;
  scan.angle_max = kTwoPi - inc;
  scan.angle_increment = inc;
  scan.range_min = kScanRangeMin;
  scan.range_max = kScanRangeMax;
  scan.ranges = std::move(hits);
  if (scan_callback_) {
    scan_callback_(scan);
  }
}

void WafflePiArm::sample_imu(double time, double imu_dt) {
  // 여기서는 진실 IMU만 만든다. 실제 센서 잡음은 SimulatorOpenCr이 입힌다.
  const double true_linear = last_linear_velocity_;
  const double true_angular = last_angular_velocity_;
  const double forward_accel =
      imu_dt > 0.0 ? (true_linear - last_imu_linear_velocity_) / imu_dt : 0.0;
  const double lateral_accel = true_linear * true_angular;
  last_imu_linear_velocity_ = true_linear;
  const double gravity = world != nullptr ? world->gravity() : 9.81;

  msgs::Imu imu;
  imu.header.stamp = time;
  imu.header.frame_id = "imu_link";
  imu.orientation = tfmath::from_yaw(pose.theta);
  imu.angular_velocity = {0.0, 0.0, true_angular};
  imu.linear_acceleration = {forward_accel, lateral_accel, gravity};
  if (imu_callback_) {
    imu_callback_(imu);
  }
}

void WafflePiArm::set_manual_arm(bool manual) {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (manual_ == manual) {
    return;
  }
  manual_ = manual;
  if (manual_) {
    manual_position_ = controller_position_;
    manual_target_ = manual_position_;
  } else {
    controller_position_ = manual_position_;
    controller_target_ = controller_position_;
  }
}

bool WafflePiArm::manual_arm() const {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  return manual_;
}

void WafflePiArm::set_arm_target(const std::array<double, 5>& target) {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  manual_ = true;
  manual_target_ = target;
}

void WafflePiArm::set_controller_arm_target(const std::array<double, 5>& target) {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (!manual_) {
    controller_target_ = target;
  }
}

ArmStatus WafflePiArm::arm_status() const {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  ArmStatus status;
  status.position = manual_ ? manual_position_ : controller_position_;
  status.present = true;
  return status;
}

std::string WafflePiArm::noise_summary() const {
  if (!noisy_) {
    return "off (--no-noise)";
  }
  char buffer[192];
  std::snprintf(buffer, sizeof(buffer),
                "wheel slip sigma=%.3f m/s scale=%+.1f%%/%+.1f%% (physical)",
                left_slip_.sigma, left_slip_.scale * 100.0, right_slip_.scale * 100.0);
  return buffer;
}

void WafflePiArm::step_towards(std::array<double, 5>& position,
                               const std::array<double, 5>& target, double dt) {
  for (std::size_t index = 0; index < position.size(); ++index) {
    const double max_step = (index == 4 ? 0.03 : 1.2) * dt;
    const double delta = std::clamp(target[index] - position[index], -max_step, max_step);
    position[index] += delta;
  }
}

krosc::simulator::RobotState WafflePiArm::state() const {
  krosc::simulator::RobotState result;
  result.command = command();
  result.joints = joint_positions();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    result.scan_ranges = truth_scan_ranges_;
    result.scan_pose = truth_scan_pose_;
  }
  if (visualizer_) {
    result.visuals = visualizer_->visuals(pose, result.joints);
  }
  return result;
}

}  // namespace krosc_ws::sim
