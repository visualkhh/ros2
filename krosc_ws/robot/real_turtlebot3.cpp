#include "real_turtlebot3.hpp"

#include <stdexcept>
#include <utility>

#include "controls/opencr/opencr_control.hpp"
#include "sensors/lds03/lds03_sensor.hpp"
#include "sensors/opencr/opencr_imu.hpp"

namespace krosc_ws {

namespace {

krosc::hardware::HardwareConfig hw_config(std::string name, double read_period,
                                          double write_period) {
  krosc::hardware::HardwareConfig config;
  config.name = std::move(name);
  config.read_period = read_period;
  config.write_period = write_period;
  return config;
}

}  // namespace

RealTurtleBot3::RealTurtleBot3(krosc::Context& context, std::string opencr_port,
                               std::string lidar_port, bool enable_lidar, HardwareOptions options)
    : context_(context),
      opencr_(kWheelRadius, kWheelSeparation, std::move(opencr_port), 115200, 0.02, 0.5, -1.0, -1.0,
              kMaxWheel) {
  // 이 로봇은 두 모터 모두 +속도 = 뒤로 (robot.yaml left_dir/right_dir = -1).
  auto control = std::make_unique<OpenCrControl>(opencr_, kWheelRadius, kWheelSeparation);
  control_ = control.get();
  if (!hardware_.add(std::move(control),
                     hw_config("opencr_control", options.control_read_period,
                               options.control_write_period))) {
    throw std::runtime_error("RealTurtleBot3: OpenCR control 등록 실패");
  }

  auto imu = std::make_unique<OpenCrImu>(context_, opencr_, options.imu_topic, options.imu_frame);
  if (!hardware_.add(std::move(imu), hw_config("opencr_imu", options.imu_period, 0.0))) {
    throw std::runtime_error("RealTurtleBot3: IMU 등록 실패 (imu 주기 확인)");
  }

  if (enable_lidar) {
    lidar_ = std::make_unique<Lds03Device>(std::move(lidar_port), kLidarMountX, true, true);
    auto scan = std::make_unique<Lds03Sensor>(context_, *lidar_, options.scan_topic,
                                              options.scan_frame);
    if (!hardware_.add(std::move(scan), hw_config("lds03_scan", options.scan_period, 0.0))) {
      throw std::runtime_error("RealTurtleBot3: LDS-03 등록 실패 (scan 주기 확인)");
    }
  }
  options_ = options;
  // TurtleBot3Driver가 부팅 직후 팔 토크/명령을 보낼 수 있도록 시리얼은 먼저 연다.
  // HardwareManager.start()의 OpenCrControl::start()는 idempotent하다.
  opencr_.start();
}

RealTurtleBot3::~RealTurtleBot3() { stop(); }

bool RealTurtleBot3::start(std::function<void(double time, double period)> control_tick) {
  if (started_) {
    return false;
  }
  if (control_tick) {
    hardware_.set_control_hook(std::move(control_tick), options_.control_period);
  }
  if (!hardware_.start()) {
    return false;
  }
  started_ = true;
  return true;
}

void RealTurtleBot3::stop() {
  if (!started_) {
    return;
  }
  hardware_.stop();
  started_ = false;
}

}  // namespace krosc_ws
