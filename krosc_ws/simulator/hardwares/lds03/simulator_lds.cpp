#include "hardwares/lds03/simulator_lds.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <random>

namespace krosc_ws::sim {

// 라이다 거리 잡음/드롭아웃은 WafflePiArm 모델이 만든다.
// 이 어댑터는 실기 LDS-03처럼 값만 전달한다.
SimulatorLds::SimulatorLds(WafflePiArm& robot, bool noisy, std::uint32_t seed)
    : robot_(robot), noisy_(noisy), rng_(seed) {
  robot_.set_scan_callback([this](const krosc::msgs::LaserScan& scan) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan_ = scan;
    ++scans_;
    last_scan_ = std::chrono::steady_clock::now();
    has_scan_ = true;
  });
}

std::optional<krosc::msgs::LaserScan> SimulatorLds::take_scan(double stamp) {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (!scan_) {
    return std::nullopt;
  }
  auto scan = std::move(scan_);
  scan_.reset();
  scan->header.stamp = stamp;
  if (noisy_) {
    std::normal_distribution<double> normal(0.0, 1.0);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    for (auto& range : scan->ranges) {
      if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
        continue;
      }
      if (uniform(rng_) < dropout_) {
        range = std::numeric_limits<float>::infinity();
        continue;
      }
      range = static_cast<float>(static_cast<double>(range) + normal(rng_) * range_sigma_);
    }
  }
  return scan;
}

double SimulatorLds::scan_age() const {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (!has_scan_) {
    return 1e9;
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_scan_).count();
}

std::string SimulatorLds::noise_summary() const {
  if (!noisy_) {
    return "off";
  }
  char buffer[128];
  std::snprintf(buffer, sizeof(buffer), "range sigma=%.3f m dropout=%.2f%%", range_sigma_,
                dropout_ * 100.0);
  return buffer;
}

}  // namespace krosc_ws::sim
