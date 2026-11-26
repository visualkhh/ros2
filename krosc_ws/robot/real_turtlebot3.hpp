#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/sensor/sensor_manager.hpp"
#include "hardwares/sensors/lds03/lds03.hpp"
#include "hardwares/controls/opencr/opencr_serial.hpp"

#ifndef KROSC_WS_URDF_PATH
#define KROSC_WS_URDF_PATH "robot/description/urdf/turtlebot3_real.urdf"
#endif

namespace krosc_ws {

// 이 프로젝트의 실기 TurtleBot3 Waffle Pi 개조형.
// OpenCR: 115200, C/S/I/E 프로토콜. LDS-03: 230400, reverse=True, 몸통 필터.
// 월드/시뮬레이션은 쓰지 않는다. 실시간 루프가 tick()을 부른다.
class RealTurtleBot3 {
 public:
  static constexpr double kWheelRadius = 0.033;       // m
  static constexpr double kWheelSeparation = 0.287;   // m (실측)
  static constexpr double kMaxWheel = 7.8;            // rad/s
  static constexpr double kLidarMountX = 0.060;       // m, base_link 기준
  static constexpr double kLidarMountHeight = 0.022;  // m, 1층 판 위
  static constexpr double kBodyRadius = 0.22;         // m, 충돌/코스트맵 반지름

  RealTurtleBot3(std::string opencr_port, std::string lidar_port, bool enable_lidar = true);
  ~RealTurtleBot3();

  RealTurtleBot3(const RealTurtleBot3&) = delete;
  RealTurtleBot3& operator=(const RealTurtleBot3&) = delete;

  void tick(double time, double period);
  void stop();

  [[nodiscard]] OpenCrSerial& opencr() { return opencr_; }
  [[nodiscard]] const OpenCrSerial& opencr() const { return opencr_; }
  [[nodiscard]] krosc::sensor::SensorManager& sensors() { return sensors_; }
  [[nodiscard]] const krosc::sensor::SensorManager& sensors() const { return sensors_; }
  [[nodiscard]] Lds03* lidar() { return lidar_; }
  [[nodiscard]] const Lds03* lidar() const { return lidar_; }
  [[nodiscard]] bool has_lidar() const { return lidar_ != nullptr; }

  [[nodiscard]] std::map<std::string, double> joint_positions() const;
  [[nodiscard]] std::optional<krosc::msgs::LaserScan> scan(double stamp);
  [[nodiscard]] std::string urdf_path() const { return KROSC_WS_URDF_PATH; }
  [[nodiscard]] std::string name() const { return "turtlebot3_waffle_pi"; }

 private:
  OpenCrSerial opencr_;
  krosc::sensor::SensorManager sensors_;
  Lds03* lidar_ = nullptr;
};

}  // namespace krosc_ws
