#pragma once

#include <map>
#include <string>

#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/topic.hpp"

#include "robot/real_turtlebot3.hpp"

namespace krosc_ws {

// 실기 하드웨어와 krosc 토픽을 잇는 접착제.
// cmd_vel -> OpenCR.write, OpenCR/LDS-03 값 -> scan/imu/joint_states/diagnostics.
// 센서 발행은 메인 실시간 루프가 poll()로 당긴다 (발행 스레드를 따로 두지 않는다).
class RobotBridge final : public krosc::Node {
 public:
  RobotBridge(krosc::Context& context, RealTurtleBot3& robot);

  void poll();

 private:
  void publish_joint_states();
  void publish_diagnostics();

  RealTurtleBot3& robot_;
  krosc::Parameter<double> joint_states_period_;
  krosc::Parameter<double> diagnostics_period_;
  krosc::Parameter<double> data_timeout_;
  double last_joint_states_ = 0.0;
  double last_diagnostics_ = 0.0;
  krosc::Subscription<krosc::msgs::Twist> command_subscription_;
};

}  // namespace krosc_ws
