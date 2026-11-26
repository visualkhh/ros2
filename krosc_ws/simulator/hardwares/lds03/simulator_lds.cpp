#include "hardwares/lds03/simulator_lds.hpp"

#include <chrono>
#include <mutex>

namespace krosc_ws::sim {

// 라이다 거리 잡음/드롭아웃은 WafflePiArm 모델이 만든다.
// 이 어댑터는 실기 LDS-03처럼 값만 전달한다.
SimulatorLds::SimulatorLds(WafflePiArm& robot) : robot_(robot) {
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
  return scan;
}

double SimulatorLds::scan_age() const {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (!has_scan_) {
    return 1e9;
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_scan_).count();
}

}  // namespace krosc_ws::sim
