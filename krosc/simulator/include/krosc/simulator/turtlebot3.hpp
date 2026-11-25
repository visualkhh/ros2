#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/simulator/world.hpp"

namespace krosc::simulator {

struct Noise {
  double sigma = 0.0;
  double bias = 0.0;
  double scale = 0.0;

  [[nodiscard]] double apply(double value, std::mt19937& rng) const {
    if (sigma == 0.0 && bias == 0.0 && scale == 0.0) {
      return value;
    }
    std::normal_distribution<double> normal(0.0, 1.0);
    return value * (1.0 + scale) + bias + normal(rng) * sigma;
  }

  [[nodiscard]] double variance() const { return sigma * sigma + bias * bias; }
};

// ROBOTIS TurtleBot3 waffle pi의 시뮬레이션 모델: 차동 구동, LDS-01 라이다, MPU-9250 IMU.
// krosc 토픽과 무관하고 월드의 시계(tick)만 받는다. 다리(RobotBridge)가 이 값을 토픽으로 옮긴다.
class TurtleBot3 final : public WorldObject {
 public:
  explicit TurtleBot3(bool noisy = false, std::optional<std::uint32_t> seed = std::nullopt);

  void set_command(const msgs::Twist& command);
  [[nodiscard]] msgs::Twist command() const;
  void tick(double time, double dt) override;
  [[nodiscard]] std::string kind() const override { return "turtlebot3"; }

  [[nodiscard]] std::map<std::string, double> joint_positions() const;
  [[nodiscard]] msgs::Pose2D scan_pose() const;

  void set_scan_callback(std::function<void(const msgs::LaserScan&)> callback);
  void set_imu_callback(std::function<void(const msgs::Imu&)> callback);

  static constexpr double kWheelRadius = 0.033;
  static constexpr double kWheelSeparation = 0.288;
  static constexpr double kBodyRadius = 0.22;
  static constexpr double kBodyHeight = 0.141;
  static constexpr double kLidarMountX = -0.064;
  static constexpr double kLidarMountHeight = 0.132;
  static constexpr int kScanBeams = 360;
  static constexpr double kScanPeriod = 0.1;
  static constexpr double kScanRangeMin = 0.12;
  static constexpr double kScanRangeMax = 3.5;
  static constexpr double kImuPeriod = 0.01;
  static constexpr int kEncoderTicks = 4096;

 private:
  void sample_scan(double time);
  void sample_imu(double time, double imu_dt);
  void apply_motion(double dt, double linear, double angular);

  mutable std::mutex command_mutex_;
  msgs::Twist command_;

  bool noisy_ = false;
  std::mt19937 rng_;
  Noise left_slip_{0.05, 0.0, -0.01};
  Noise right_slip_{0.05, 0.0, 0.015};
  Noise gyro_{0.01, 0.005, 0.0};
  Noise accel_{0.05, 0.02, 0.0};
  Noise lidar_range_{0.01, 0.0, 0.0};
  double lidar_dropout_ = 0.01;
  double gyro_bias_walk_ = 2e-5;

  double left_angle_ = 0.0;
  double right_angle_ = 0.0;
  double left_position_ = 0.0;
  double right_position_ = 0.0;
  double left_velocity_ = 0.0;
  double right_velocity_ = 0.0;

  double imu_accum_ = 0.0;
  double scan_accum_ = 0.0;
  double yaw_error_ = 0.0;
  double bias_walk_ = 0.0;
  double integrated_time_ = 0.0;
  double last_linear_velocity_ = 0.0;
  double last_angular_velocity_ = 0.0;
  double last_imu_linear_velocity_ = 0.0;

  std::function<void(const msgs::LaserScan&)> scan_callback_;
  std::function<void(const msgs::Imu&)> imu_callback_;
};

}  // namespace krosc::simulator
