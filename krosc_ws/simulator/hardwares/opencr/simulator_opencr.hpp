#pragma once

#include <array>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include "hardwares/opencr/opencr_device.hpp"
#include "robot/waffle_pi_arm.hpp"

namespace krosc_ws::sim {

// 시뮬레이터 로봇을 OpenCR 실기 장치처럼 보이게 하는 어댑터.
class SimulatorOpenCr final : public OpenCrDevice {
 public:
  explicit SimulatorOpenCr(WafflePiArm& robot);

  void start() override {}
  void stop() override {}
  [[nodiscard]] bool connected() const override { return true; }

  void write(const krosc::msgs::Twist& command) override;
  void write_arm(const std::array<double, 5>& position) override;
  void set_arm_torque(bool enabled) override;
  void buzzer(int frequency_hz, int duration_ms) override;
  void reboot_arm(const std::array<bool, 5>& joints) override;
  void tick() override {}

  [[nodiscard]] std::optional<krosc::msgs::Imu> consume_imu() override;
  [[nodiscard]] ArmStatus arm_status() const override;
  [[nodiscard]] double left_position() const override;
  [[nodiscard]] double right_position() const override;
  [[nodiscard]] double left_velocity() const override;
  [[nodiscard]] double right_velocity() const override;
  [[nodiscard]] std::optional<BoardStatus> status() const override { return std::nullopt; }
  [[nodiscard]] double status_age() const override;
  [[nodiscard]] std::uint64_t error_count() const override { return 0; }
  [[nodiscard]] std::uint64_t bad_lines() const override { return 0; }
  [[nodiscard]] std::string last_error() const override { return {}; }
  [[nodiscard]] const std::string& port() const override { return port_; }

 private:
  WafflePiArm& robot_;
  std::string port_ = "sim";
  mutable std::mutex imu_mutex_;
  std::optional<krosc::msgs::Imu> imu_;
};

}  // namespace krosc_ws::sim
