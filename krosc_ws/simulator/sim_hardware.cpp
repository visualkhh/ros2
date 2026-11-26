#include "sim_hardware.hpp"

#include <chrono>
#include <utility>

namespace krosc_ws::sim {

SimOpenCr::SimOpenCr(krosc::simulator::TurtleBot3& robot) : robot_(robot) {
  robot_.set_imu_callback([this](const krosc::msgs::Imu& imu) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_ = imu;
  });
}

void SimOpenCr::write(const krosc::msgs::Twist& command) { robot_.set_command(command); }
void SimOpenCr::write_arm(const std::array<double, 5>& position) {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  arm_position_ = position;
}
void SimOpenCr::set_arm_torque(bool) {}
void SimOpenCr::buzzer(int, int) {}
void SimOpenCr::reboot_arm(const std::array<bool, 5>&) {}

std::optional<krosc::msgs::Imu> SimOpenCr::consume_imu() {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  auto imu = std::move(imu_);
  imu_.reset();
  return imu;
}

ArmStatus SimOpenCr::arm_status() const {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  ArmStatus status;
  status.position = arm_position_;
  status.present = true;
  return status;
}

double SimOpenCr::left_position() const { return robot_.left_position(); }
double SimOpenCr::right_position() const { return robot_.right_position(); }
double SimOpenCr::left_velocity() const { return robot_.left_velocity(); }
double SimOpenCr::right_velocity() const { return robot_.right_velocity(); }
double SimOpenCr::status_age() const { return 0.0; }

SimLds::SimLds(krosc::simulator::TurtleBot3& robot) : robot_(robot) {
  robot_.set_scan_callback([this](const krosc::msgs::LaserScan& scan) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan_ = scan;
    ++scans_;
    last_scan_ = std::chrono::steady_clock::now();
    has_scan_ = true;
  });
}

std::optional<krosc::msgs::LaserScan> SimLds::take_scan(double stamp) {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (!scan_) {
    return std::nullopt;
  }
  auto scan = std::move(scan_);
  scan_.reset();
  scan->header.stamp = stamp;
  return scan;
}

double SimLds::scan_age() const {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (!has_scan_) {
    return 1e9;
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_scan_).count();
}

}  // namespace krosc_ws::sim
