#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "krosc/context.hpp"
#include "krosc/msgs.hpp"
#include "krosc/sensor/sensor_manager.hpp"
#include "hardwares/lds03/sensor/lds03.hpp"
#include "hardwares/opencr/opencr_serial.hpp"

#ifndef KROSC_WS_URDF_PATH
#define KROSC_WS_URDF_PATH "robot/description/urdf/turtlebot3_real.urdf"
#endif

namespace krosc_ws {

// 센서별 발행 토픽/프레임/주기. 기본값은 기존 동작(scan 10Hz, imu 50Hz)과 같다.
struct SensorOptions {
  std::string scan_topic = "scan";
  std::string scan_frame = "base_scan";
  double scan_period = 0.1;
  std::string imu_topic = "imu";
  std::string imu_frame = "imu_link";
  double imu_period = 0.02;
  std::size_t workers = 10;  // SensorManager on_update 워커 수
};

// 이 프로젝트의 실기 TurtleBot3 Waffle Pi 개조형.
// OpenCR: 115200, C/S/I/E 프로토콜. LDS-03: 230400, reverse=True, 몸통 필터.
// 월드/시뮬레이션은 쓰지 않는다. 실시간 루프가 tick()을 부른다.
// 센서는 SensorManager가 자기 주기로 읽어 설정된 토픽으로 발행한다.
class RealTurtleBot3 {
 public:
  static constexpr double kWheelRadius = 0.033;       // m
  static constexpr double kWheelSeparation = 0.287;   // m (실측)
  static constexpr double kMaxWheel = 7.8;            // rad/s
  static constexpr double kLidarMountX = 0.060;       // m, base_link 기준
  static constexpr double kLidarMountHeight = 0.022;  // m, 1층 판 위
  static constexpr double kBodyRadius = 0.22;         // m, 충돌/코스트맵 반지름

  RealTurtleBot3(krosc::Context& context, std::string opencr_port, std::string lidar_port,
                 bool enable_lidar = true, SensorOptions options = {});
  ~RealTurtleBot3();

  RealTurtleBot3(const RealTurtleBot3&) = delete;
  RealTurtleBot3& operator=(const RealTurtleBot3&) = delete;

  void tick();
  void stop();

  [[nodiscard]] OpenCrSerial& opencr() { return opencr_; }
  [[nodiscard]] const OpenCrSerial& opencr() const { return opencr_; }
  [[nodiscard]] krosc::sensor::SensorManager& sensors() { return sensors_; }
  [[nodiscard]] const krosc::sensor::SensorManager& sensors() const { return sensors_; }
  [[nodiscard]] Lds03* lidar() { return lidar_; }
  [[nodiscard]] const Lds03* lidar() const { return lidar_; }
  [[nodiscard]] bool has_lidar() const { return lidar_ != nullptr; }

  [[nodiscard]] std::map<std::string, double> joint_positions() const;
  [[nodiscard]] std::string urdf_path() const { return KROSC_WS_URDF_PATH; }
  [[nodiscard]] std::string name() const { return "turtlebot3_waffle_pi"; }

 private:
  krosc::Context& context_;
  OpenCrSerial opencr_;
  krosc::sensor::SensorManager sensors_;
  Lds03* lidar_ = nullptr;
};

}  // namespace krosc_ws
