#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "hardwares/lds03/lds_device.hpp"
#include "robot/waffle_pi_arm.hpp"

namespace krosc_ws::sim {

// 시뮬레이터 스캔을 LDS-03 실기 장치처럼 보이게 하는 어댑터.
class SimulatorLds final : public LdsDevice {
 public:
  explicit SimulatorLds(WafflePiArm& robot);

  void start() override {}
  void stop() override {}
  [[nodiscard]] bool connected() const override { return true; }

  [[nodiscard]] std::optional<krosc::msgs::LaserScan> take_scan(double stamp) override;
  [[nodiscard]] std::uint64_t scans() const override { return scans_; }
  [[nodiscard]] std::uint64_t bad_packets() const override { return 0; }
  [[nodiscard]] double scan_age() const override;
  [[nodiscard]] const std::string& port() const override { return port_; }

 private:
  WafflePiArm& robot_;
  std::string port_ = "sim";
  mutable std::mutex scan_mutex_;
  std::optional<krosc::msgs::LaserScan> scan_;
  std::uint64_t scans_ = 0;
  std::chrono::steady_clock::time_point last_scan_{};
  bool has_scan_ = false;
};

}  // namespace krosc_ws::sim
