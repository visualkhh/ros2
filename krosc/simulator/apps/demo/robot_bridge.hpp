#pragma once

#include <string>

#include "krosc/node.hpp"
#include "krosc/simulator/robots/turtlebot/turtlebot.hpp"
#include "krosc/time/timer.hpp"

namespace krosc::simulator {

// 시뮬레이션 로봇과 krosc 토픽을 잇는 접착제 노드 (실제 로봇의 하드웨어 드라이버 자리).
// cmd_vel -> robot.command, 라이다/IMU 측정 -> scan/imu, 바퀴 위치 -> joint_states.
class RobotBridge final : public krosc::Node {
 public:
  RobotBridge(krosc::Context& context, krosc::simulator::TurtleBot3& robot);

 private:
  void publish_joint_states();

  krosc::simulator::TurtleBot3& robot_;
  krosc::Timer joint_states_timer_;
};

}  // namespace krosc::simulator
