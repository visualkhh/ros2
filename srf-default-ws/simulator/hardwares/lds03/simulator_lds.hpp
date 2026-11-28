#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <random>
#include <string>

#include "hardwares/lds03/lds_device.hpp"
#include "robot/waffle_pi_arm.hpp"

namespace srf_default_ws::sim {

// 시뮬레이터 스캔을 LDS-03 실기 장치처럼 보이게 하는 어댑터.
class SimulatorLds final : public LdsDevice {
 public:
  explicit SimulatorLds(WafflePiArm& robot, bool noisy = true, std::uint32_t seed = 42);

  void start() override { connected_.store(true); }
  void stop() override { connected_.store(false); }
  void connect() override { connected_.store(true); }
  void disconnect() override { connected_.store(false); }
  [[nodiscard]] bool is_connected() const override { return connected_.load(); }

  [[nodiscard]] std::optional<srf::msgs::LaserScan> take_scan(double stamp) override;
  [[nodiscard]] std::uint64_t scans() const override { return scans_; }
  [[nodiscard]] std::uint64_t bad_packets() const override { return 0; }
  [[nodiscard]] double scan_age() const override;
  [[nodiscard]] const std::string& port() const override { return port_; }
  [[nodiscard]] std::string noise_summary() const;

 private:
  WafflePiArm& robot_;
  bool noisy_ = true;
  std::atomic<bool> connected_{true};
  std::mt19937 rng_;
  double range_sigma_ = 0.004;
  double dropout_ = 0.003;
  std::string port_ = "sim";
  mutable std::mutex scan_mutex_;
  std::optional<srf::msgs::LaserScan> scan_;
  std::uint64_t scans_ = 0;
  std::chrono::steady_clock::time_point last_scan_{};
  bool has_scan_ = false;
};

}  // namespace srf_default_ws::sim
