#pragma once

#include <array>
#include <map>
#include <string>

#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/topic.hpp"

namespace krosc_ws {

class OpenCrDevice;
class LdsDevice;

// OpenCR/LDS-03의 논리 상태/서비스 노드.
// 센서 데이터는 HardwareManager가 발행하고, 여기서는 arm_status/diagnostics와
// 팔 토크/정지/리셋/부저 서비스를 담당한다.
class RobotStatus final : public krosc::Node {
 public:
  RobotStatus(krosc::Context& context, OpenCrDevice& opencr, LdsDevice* lidar);

  void poll();

 private:
  void publish_diagnostics();
  void publish_arm_status();

  OpenCrDevice& opencr_;
  LdsDevice* lidar_;
  krosc::Parameter<double> diagnostics_period_;
  krosc::Parameter<double> data_timeout_;
  krosc::Parameter<bool> arm_torque_on_start_;
  krosc::Timer poll_timer_;
  double last_diagnostics_ = 0.0;
  double last_arm_status_ = 0.0;
};

}  // namespace krosc_ws
