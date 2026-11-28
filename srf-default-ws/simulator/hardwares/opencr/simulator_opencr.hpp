#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>

#include "hardwares/opencr/opencr_device.hpp"
#include "hardwares/opencr/simulator_buzzer.hpp"
#include "robot/waffle_pi_arm.hpp"

namespace srf_default_ws::sim {

// 시뮬레이터 로봇을 OpenCR 실기 장치처럼 보이게 하는 어댑터.
class SimulatorOpenCr final : public OpenCrDevice {
 public:
  explicit SimulatorOpenCr(WafflePiArm& robot, bool noisy = true, std::uint32_t seed = 42);

  void start() override { connected_.store(true); }
  void stop() override { connected_.store(false); }
  void connect() override { connected_.store(true); }
  void disconnect() override { connected_.store(false); }
  [[nodiscard]] bool is_connected() const override { return connected_.load(); }

  void write(const srf::msgs::Twist& command) override;
  void write_arm(const std::array<double, 5>& position) override;
  void set_arm_torque(bool enabled) override;
  void buzzer(int frequency_hz, int duration_ms) override;
  void reboot_arm(const std::array<bool, 5>& joints) override;
  void tick() override {}

  [[nodiscard]] std::optional<srf::msgs::Imu> consume_imu() override;
  [[nodiscard]] ArmStatus arm_status() const override;
  [[nodiscard]] double left_position() const override;
  [[nodiscard]] double right_position() const override;
  [[nodiscard]] double left_velocity() const override;
  [[nodiscard]] double right_velocity() const override;
  [[nodiscard]] std::optional<BoardStatus> status() const override;
  [[nodiscard]] double status_age() const override;
  [[nodiscard]] std::uint64_t error_count() const override { return 0; }
  [[nodiscard]] std::uint64_t bad_lines() const override { return 0; }
  [[nodiscard]] std::string last_error() const override { return {}; }
  [[nodiscard]] const std::string& port() const override { return port_; }
  [[nodiscard]] std::string noise_summary() const;

 private:
  struct Noise {
    double sigma = 0.0;
    double bias = 0.0;
    double scale = 0.0;

    [[nodiscard]] double apply(double value, std::mt19937& rng) const {
      std::normal_distribution<double> normal(0.0, 1.0);
      return value * (1.0 + scale) + bias + normal(rng) * sigma;
    }
  };

  WafflePiArm& robot_;
  bool noisy_ = true;
  std::atomic<bool> connected_{true};
  std::mt19937 rng_;
  Noise gyro_{0.002, 0.0015, 0.0};
  Noise accel_{0.03, 0.01, 0.0};
  double gyro_bias_walk_ = 2e-5;
  double bias_walk_ = 0.0;
  double yaw_error_ = 0.0;
  double integrated_time_ = 0.0;
  double last_imu_stamp_ = -1.0;
  std::string port_ = "sim";
  std::unique_ptr<SimulatorBuzzer> buzzer_;
  std::chrono::steady_clock::time_point started_{std::chrono::steady_clock::now()};
  mutable std::mutex imu_mutex_;
  std::optional<srf::msgs::Imu> imu_;
};

}  // namespace srf_default_ws::sim
