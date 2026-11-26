#include "krosc/simulator/robots/turtlebot/turtlebot.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "krosc/tf/math.hpp"

namespace krosc::simulator {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxSubstep = 0.05;
constexpr double kTwoPi = 2.0 * kPi;

double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

std::pair<double, double> wheel_speeds(const msgs::Twist& command, double wheel_radius, double separation) {
  const double half = separation / 2.0;
  return {(command.linear.x - command.angular.z * half) / wheel_radius,
          (command.linear.x + command.angular.z * half) / wheel_radius};
}

double quantize(double angle, int ticks_per_rev) {
  if (ticks_per_rev <= 0) {
    return angle;
  }
  const double step = kTwoPi / ticks_per_rev;
  return std::round(angle / step) * step;
}

RobotVisual box_visual(const msgs::Transform& transform, std::array<double, 3> size,
                       std::array<double, 3> color) {
  RobotVisual visual;
  visual.type = RobotVisualType::Box;
  visual.transform = transform;
  visual.size = size;
  visual.color = color;
  return visual;
}

RobotVisual cylinder_visual(const msgs::Transform& transform, double radius, double length,
                            int sides, std::array<double, 3> color) {
  RobotVisual visual;
  visual.type = RobotVisualType::Cylinder;
  visual.transform = transform;
  visual.radius = radius;
  visual.length = length;
  visual.sides = sides;
  visual.color = color;
  return visual;
}

}  // namespace

TurtleBot3::TurtleBot3(bool noisy, std::optional<std::uint32_t> seed)
    : noisy_(noisy), rng_(seed ? *seed : std::random_device{}()) {
  radius = kBodyRadius;
  height = kBodyHeight;
  mass = 1.8;
  friction = 0.3;
  movable = false;
}

void TurtleBot3::set_command(const msgs::Twist& command) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_ = command;
}

msgs::Twist TurtleBot3::command() const {
  std::lock_guard<std::mutex> lock(command_mutex_);
  return command_;
}

RobotState TurtleBot3::state() const {
  RobotState result;
  result.command = command();
  result.joints = joint_positions();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    result.scan_ranges = truth_scan_ranges_;
    result.scan_pose = truth_scan_pose_;
  }

  const msgs::Transform base{{pose.x, pose.y, 0.0}, tfmath::from_yaw(pose.theta)};
  const auto local = [&](double x, double y, double z, const msgs::Quaternion& rotation = {}) {
    return tfmath::compose(base, {{x, y, z}, rotation});
  };
  result.visuals.push_back(
      box_visual(local(0.0, 0.0, 0.057), {0.266, 0.266, 0.094}, {0.765, 0.792, 0.847}));
  result.visuals.push_back(
      box_visual(local(0.073, -0.011, 0.084), {0.04, 0.03, 0.03}, {0.216, 0.227, 0.259}));
  result.visuals.push_back(
      cylinder_visual(local(-0.064, 0.0, 0.132), 0.035, 0.03, 16, {0.255, 0.267, 0.306}));
  const msgs::Quaternion wheel_rotation = tfmath::from_rpy(kPi / 2.0, 0.0, 0.0);
  for (double side : {-1.0, 1.0}) {
    result.visuals.push_back(cylinder_visual(local(0.0, side * 0.144, 0.023, wheel_rotation),
                                             0.033, 0.018, 16, {0.176, 0.184, 0.212}));
    result.visuals.push_back(box_visual(local(-0.177, side * 0.064, 0.01),
                                        {0.03, 0.009, 0.02}, {0.353, 0.369, 0.408}));
  }
  return result;
}

void TurtleBot3::tick(double time, double dt) {
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
    left_position_ = noisy_ ? quantize(left_angle_, kEncoderTicks) : left_angle_;
    right_position_ = noisy_ ? quantize(right_angle_, kEncoderTicks) : right_angle_;
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
}

void TurtleBot3::apply_motion(double dt, double linear, double angular) {
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

msgs::Pose2D TurtleBot3::scan_pose() const {
  return {pose.x + kLidarMountX * std::cos(pose.theta), pose.y + kLidarMountX * std::sin(pose.theta), pose.theta};
}

std::map<std::string, double> TurtleBot3::joint_positions() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return {{"wheel_left_joint", left_position_}, {"wheel_right_joint", right_position_}};
}

double TurtleBot3::left_position() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return left_position_;
}
double TurtleBot3::right_position() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return right_position_;
}
double TurtleBot3::left_velocity() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return left_velocity_;
}
double TurtleBot3::right_velocity() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return right_velocity_;
}

void TurtleBot3::set_scan_callback(std::function<void(const msgs::LaserScan&)> callback) {
  scan_callback_ = std::move(callback);
}

void TurtleBot3::set_imu_callback(std::function<void(const msgs::Imu&)> callback) {
  imu_callback_ = std::move(callback);
}

void TurtleBot3::sample_scan(double time) {
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
  std::vector<float> truth = hits;
  for (auto& range : truth) {
    if (!(static_cast<double>(range) >= kScanRangeMin && static_cast<double>(range) <= kScanRangeMax)) {
      range = std::numeric_limits<float>::infinity();
    }
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    truth_scan_ranges_ = std::move(truth);
    truth_scan_pose_ = {sensor_x, sensor_y, pose.theta};
  }
  for (auto& range : hits) {
    const double distance = static_cast<double>(range);
    if (!(distance >= kScanRangeMin && distance <= kScanRangeMax)) {
      range = std::numeric_limits<float>::infinity();
    } else if (noisy_) {
      std::uniform_real_distribution<double> uniform(0.0, 1.0);
      if (uniform(rng_) < lidar_dropout_) {
        range = std::numeric_limits<float>::infinity();
      } else {
        range = static_cast<float>(lidar_range_.apply(distance, rng_));
      }
    }
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

void TurtleBot3::sample_imu(double time, double imu_dt) {
  const double true_linear = last_linear_velocity_;
  const double true_angular = last_angular_velocity_;

  double measured_angular = true_angular;
  if (noisy_) {
    std::normal_distribution<double> walk(0.0, 1.0);
    bias_walk_ += walk(rng_) * gyro_bias_walk_ * std::sqrt(imu_dt);
    measured_angular = gyro_.apply(true_angular, rng_) + bias_walk_;
  }
  yaw_error_ += (measured_angular - true_angular) * imu_dt;
  integrated_time_ += imu_dt;

  const double yaw = pose.theta + yaw_error_;
  const double forward_accel = imu_dt > 0.0 ? (true_linear - last_imu_linear_velocity_) / imu_dt : 0.0;
  const double lateral_accel = true_linear * true_angular;
  last_imu_linear_velocity_ = true_linear;
  const double measured_forward_accel = noisy_ ? accel_.apply(forward_accel, rng_) : forward_accel;
  const double measured_lateral_accel = noisy_ ? accel_.apply(lateral_accel, rng_) : lateral_accel;
  const double gravity = world != nullptr ? world->gravity() : 9.81;

  msgs::Imu imu;
  imu.header.stamp = time;
  imu.header.frame_id = "imu_link";
  imu.orientation = tfmath::from_yaw(yaw);
  imu.angular_velocity = {0.0, 0.0, measured_angular};
  imu.linear_acceleration = {measured_forward_accel, measured_lateral_accel,
                             noisy_ ? accel_.apply(gravity, rng_) : gravity};
  if (noisy_) {
    const double yaw_sigma = std::abs(gyro_.bias) * integrated_time_ + gyro_.sigma * std::sqrt(integrated_time_);
    imu.orientation_covariance[8] = yaw_sigma * yaw_sigma;
    const double gyro_variance = gyro_.variance();
    imu.angular_velocity_covariance[0] = gyro_variance;
    imu.angular_velocity_covariance[4] = gyro_variance;
    imu.angular_velocity_covariance[8] = gyro_variance;
    const double accel_variance = accel_.variance();
    imu.linear_acceleration_covariance[0] = accel_variance;
    imu.linear_acceleration_covariance[4] = accel_variance;
    imu.linear_acceleration_covariance[8] = accel_variance;
  }
  if (imu_callback_) {
    imu_callback_(imu);
  }
}

}  // namespace krosc::simulator
