#include "hardwares/opencr/simulator_opencr.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <mutex>
#include <random>
#include <utility>

#include "krosc/tf/math.hpp"

namespace krosc_ws::sim {

// 바퀴 슬립/엔코더/IMU 잡음은 WafflePiArm 모델이 만든다.
// 이 어댑터는 실기 OpenCR처럼 값만 전달한다.
SimulatorOpenCr::SimulatorOpenCr(WafflePiArm& robot, bool noisy, std::uint32_t seed)
    : robot_(robot), noisy_(noisy), rng_(seed) {
  buzzer_ = std::make_unique<SimulatorBuzzer>();
  robot_.set_imu_callback([this](const krosc::msgs::Imu& imu) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_ = imu;
  });
}

void SimulatorOpenCr::write(const krosc::msgs::Twist& command) { robot_.set_command(command); }

void SimulatorOpenCr::write_arm(const std::array<double, 5>& position) {
  robot_.set_controller_arm_target(position);
}

void SimulatorOpenCr::set_arm_torque(bool) {}
void SimulatorOpenCr::buzzer(int frequency_hz, int duration_ms) {
  std::printf("[krosc_ws_sim] buzzer %d Hz %d ms\n", frequency_hz, duration_ms);
  std::fflush(stdout);
  if (buzzer_) {
    buzzer_->play(frequency_hz, duration_ms);
  }
}
void SimulatorOpenCr::reboot_arm(const std::array<bool, 5>&) {}

std::optional<krosc::msgs::Imu> SimulatorOpenCr::consume_imu() {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  auto imu = std::move(imu_);
  imu_.reset();
  if (!imu || !noisy_) {
    return imu;
  }

  const double dt = last_imu_stamp_ < 0.0 ? 0.01 : std::max(0.0, imu->header.stamp - last_imu_stamp_);
  last_imu_stamp_ = imu->header.stamp;
  integrated_time_ += dt;
  std::normal_distribution<double> walk(0.0, 1.0);
  bias_walk_ += walk(rng_) * gyro_bias_walk_ * std::sqrt(std::max(dt, 1e-6));
  const double true_angular = imu->angular_velocity.z;
  const double measured_angular = gyro_.apply(true_angular, rng_) + bias_walk_;
  yaw_error_ += (measured_angular - true_angular) * dt;
  imu->orientation = krosc::tfmath::from_yaw(krosc::tfmath::yaw_of(imu->orientation) + yaw_error_);
  imu->angular_velocity.z = measured_angular;
  imu->linear_acceleration.x = accel_.apply(imu->linear_acceleration.x, rng_);
  imu->linear_acceleration.y = accel_.apply(imu->linear_acceleration.y, rng_);
  imu->linear_acceleration.z = accel_.apply(imu->linear_acceleration.z, rng_);

  const double yaw_sigma =
      std::abs(gyro_.bias) * integrated_time_ + gyro_.sigma * std::sqrt(integrated_time_);
  imu->orientation_covariance[8] = yaw_sigma * yaw_sigma;
  const double gyro_variance = gyro_.sigma * gyro_.sigma + gyro_.bias * gyro_.bias;
  imu->angular_velocity_covariance[0] = gyro_variance;
  imu->angular_velocity_covariance[4] = gyro_variance;
  imu->angular_velocity_covariance[8] = gyro_variance;
  const double accel_variance = accel_.sigma * accel_.sigma + accel_.bias * accel_.bias;
  imu->linear_acceleration_covariance[0] = accel_variance;
  imu->linear_acceleration_covariance[4] = accel_variance;
  imu->linear_acceleration_covariance[8] = accel_variance;
  return imu;
}

ArmStatus SimulatorOpenCr::arm_status() const { return robot_.arm_status(); }

namespace {
constexpr int kEncoderTicks = 4096;

double quantize_encoder(double angle) {
  const double step = 2.0 * 3.14159265358979323846 / static_cast<double>(kEncoderTicks);
  return std::round(angle / step) * step;
}
}  // namespace

double SimulatorOpenCr::left_position() const {
  return noisy_ ? quantize_encoder(robot_.left_position()) : robot_.left_position();
}
double SimulatorOpenCr::right_position() const {
  return noisy_ ? quantize_encoder(robot_.right_position()) : robot_.right_position();
}
double SimulatorOpenCr::left_velocity() const { return robot_.left_velocity(); }
double SimulatorOpenCr::right_velocity() const { return robot_.right_velocity(); }
std::optional<BoardStatus> SimulatorOpenCr::status() const {
  BoardStatus board;
  const double left = robot_.left_velocity();
  const double right = robot_.right_velocity();
  const double load = std::abs(left) + std::abs(right);
  board.left_position = robot_.left_position();
  board.right_position = robot_.right_position();
  board.left_velocity = left;
  board.right_velocity = right;
  board.left_current = 0.18 + std::abs(left) * 0.55;
  board.right_current = 0.18 + std::abs(right) * 0.55;
  board.left_temperature = 29.0 + load * 4.0;
  board.right_temperature = 29.0 + load * 4.0;
  board.battery_voltage = std::clamp(12.4 - load * 0.12, 10.5, 12.6);
  board.uptime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started_).count();
  return board;
}

double SimulatorOpenCr::status_age() const { return 0.0; }

std::string SimulatorOpenCr::noise_summary() const {
  if (!noisy_) {
    return "off";
  }
  char buffer[192];
  std::snprintf(buffer, sizeof(buffer),
                "encoder=%d ticks gyro sigma=%.4f rad/s bias=%.4f accel sigma=%.3f m/s^2 bias=%.3f",
                kEncoderTicks, gyro_.sigma, gyro_.bias, accel_.sigma, accel_.bias);
  return buffer;
}

}  // namespace krosc_ws::sim
