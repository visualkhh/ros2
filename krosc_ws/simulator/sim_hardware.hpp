#pragma once

#include <array>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include "hardwares/lds03/lds_device.hpp"
#include "hardwares/opencr/opencr_device.hpp"
#include "krosc/simulator/turtlebot3.hpp"

namespace krosc_ws::sim {

// 시뮬레이터 로봇을 OpenCR 실기 장치처럼 보이게 하는 어댑터.
class SimOpenCr final : public OpenCrDevice {
 public:
  explicit SimOpenCr(krosc::simulator::TurtleBot3& robot);

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
  krosc::simulator::TurtleBot3& robot_;
  std::string port_ = "sim";
  mutable std::mutex imu_mutex_;
  std::optional<krosc::msgs::Imu> imu_;
  mutable std::mutex arm_mutex_;
  // 시뮬레이터 시작 자세는 MoveIt HOME과 같은 안전 자세로 둔다.
  std::array<double, 5> arm_position_{0.5, -1.8, 1.4, 0.8, 0.0};
};

// 시뮬레이터 스캔을 LDS-03 실기 장치처럼 보이게 하는 어댑터.
class SimLds final : public LdsDevice {
 public:
  explicit SimLds(krosc::simulator::TurtleBot3& robot);

  void start() override {}
  void stop() override {}
  [[nodiscard]] bool connected() const override { return true; }

  [[nodiscard]] std::optional<krosc::msgs::LaserScan> take_scan(double stamp) override;
  [[nodiscard]] std::uint64_t scans() const override { return scans_; }
  [[nodiscard]] std::uint64_t bad_packets() const override { return 0; }
  [[nodiscard]] double scan_age() const override;
  [[nodiscard]] const std::string& port() const override { return port_; }

 private:
  krosc::simulator::TurtleBot3& robot_;
  std::string port_ = "sim";
  mutable std::mutex scan_mutex_;
  std::optional<krosc::msgs::LaserScan> scan_;
  std::uint64_t scans_ = 0;
  std::chrono::steady_clock::time_point last_scan_{};
  bool has_scan_ = false;
};

}  // namespace krosc_ws::sim
