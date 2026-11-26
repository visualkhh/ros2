#pragma once

#include <array>
#include <map>
#include <string>

#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/topic.hpp"

#include "real_turtlebot3.hpp"

namespace krosc_ws {

// 실기 하드웨어와 krosc 토픽/서비스를 잇는 접착제.
// OpenCR/LDS-03 센서 데이터는 SensorManager가 자기 주기로 발행하고,
// 여기서는 arm_status/diagnostics 발행과 팔/부저 서비스를 담당한다.
// cmd_vel은 control diff_drive_controller, joint_states는 joint_state_broadcaster가 낸다.
// 진단/팔 상태 발행은 메인 실시간 루프가 poll()로 당긴다 (발행 스레드를 따로 두지 않는다).
class TurtleBot3Driver final : public krosc::Node {
 public:
  TurtleBot3Driver(krosc::Context& context, RealTurtleBot3& robot);

  void poll();

 private:
  void publish_diagnostics();
  void publish_arm_status();

  RealTurtleBot3& robot_;
  krosc::Parameter<double> diagnostics_period_;
  krosc::Parameter<double> data_timeout_;
  krosc::Parameter<bool> arm_torque_on_start_;
  double last_diagnostics_ = 0.0;
  double last_arm_status_ = 0.0;
};

}  // namespace krosc_ws
