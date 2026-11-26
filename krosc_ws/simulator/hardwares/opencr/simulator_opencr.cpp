#include "hardwares/opencr/simulator_opencr.hpp"

#include <mutex>
#include <utility>

namespace krosc_ws::sim {

// 바퀴 슬립/엔코더/IMU 잡음은 WafflePiArm(TurtleBot3) 모델이 만든다.
// 이 어댑터는 실기 OpenCR처럼 값만 전달한다.
SimulatorOpenCr::SimulatorOpenCr(WafflePiArm& robot) : robot_(robot) {
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
void SimulatorOpenCr::buzzer(int, int) {}
void SimulatorOpenCr::reboot_arm(const std::array<bool, 5>&) {}

std::optional<krosc::msgs::Imu> SimulatorOpenCr::consume_imu() {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  auto imu = std::move(imu_);
  imu_.reset();
  return imu;
}

ArmStatus SimulatorOpenCr::arm_status() const { return robot_.arm_status(); }

double SimulatorOpenCr::left_position() const { return robot_.left_position(); }
double SimulatorOpenCr::right_position() const { return robot_.right_position(); }
double SimulatorOpenCr::left_velocity() const { return robot_.left_velocity(); }
double SimulatorOpenCr::right_velocity() const { return robot_.right_velocity(); }
double SimulatorOpenCr::status_age() const { return 0.0; }

}  // namespace krosc_ws::sim
