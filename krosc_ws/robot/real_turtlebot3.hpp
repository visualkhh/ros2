#pragma once

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include "krosc/context.hpp"
#include "krosc/hardware/hardware_manager.hpp"

#include "hardwares/lds03/lds03_device.hpp"
#include "hardwares/opencr/opencr_serial.hpp"

#ifndef KROSC_WS_URDF_PATH
#define KROSC_WS_URDF_PATH "robot/description/urdf/turtlebot3_real.urdf"
#endif

namespace krosc_ws {

class OpenCrControl;

// 하드웨어/논리 레이어 설정. 주기는 HardwareManager 등록 시 사용한다.
struct HardwareOptions {
  std::string scan_topic = "scan";
  std::string scan_frame = "base_scan";
  double scan_period = 0.1;
  std::string imu_topic = "imu";
  std::string imu_frame = "imu_link";
  double imu_period = 0.02;
  double control_read_period = 0.02;
  double control_write_period = 0.02;
  double control_period = 0.01;
};

// 이 프로젝트의 실기 TurtleBot3 Waffle Pi 개조형.
//   hardwares: OpenCrSerial, Lds03Device (물리 장치)
//   controls : OpenCrControl (state/command interface)
//   sensors  : OpenCrImu, Lds03Sensor (토픽 발행)
// HardwareManager가 read → control hook → write 순서로 돌린다.
class RealTurtleBot3 {
 public:
  static constexpr double kWheelRadius = 0.033;       // m
  static constexpr double kWheelSeparation = 0.287;   // m (실측)
  static constexpr double kMaxWheel = 7.8;            // rad/s
  static constexpr double kLidarMountX = 0.060;       // m, base_link 기준
  static constexpr double kLidarMountHeight = 0.022;  // m, 1층 판 위
  static constexpr double kBodyRadius = 0.22;         // m, 충돌/코스트맵 반지름

  RealTurtleBot3(krosc::Context& context, std::string opencr_port, std::string lidar_port,
                 bool enable_lidar = true, HardwareOptions options = {});
  ~RealTurtleBot3();

  RealTurtleBot3(const RealTurtleBot3&) = delete;
  RealTurtleBot3& operator=(const RealTurtleBot3&) = delete;

  // control_tick은 read와 write 사이에 호출된다 (보통 controller_manager.update).
  bool start(std::function<void(double time, double period)> control_tick = {});
  void stop();

  [[nodiscard]] OpenCrSerial& opencr() { return opencr_; }
  [[nodiscard]] const OpenCrSerial& opencr() const { return opencr_; }
  [[nodiscard]] OpenCrControl* control() { return control_; }
  [[nodiscard]] const OpenCrControl* control() const { return control_; }
  [[nodiscard]] Lds03Device* lidar() { return lidar_.get(); }
  [[nodiscard]] const Lds03Device* lidar() const { return lidar_.get(); }
  [[nodiscard]] bool has_lidar() const { return lidar_ != nullptr; }

  [[nodiscard]] std::string urdf_path() const { return KROSC_WS_URDF_PATH; }
  [[nodiscard]] std::string name() const { return "turtlebot3_waffle_pi"; }

 private:
  krosc::Context& context_;
  OpenCrSerial opencr_;
  std::unique_ptr<Lds03Device> lidar_;
  krosc::hardware::HardwareManager hardware_;
  OpenCrControl* control_ = nullptr;
  HardwareOptions options_;
  bool started_ = false;
};

}  // namespace krosc_ws
