#pragma once

#include <map>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/simulator/world.hpp"

namespace krosc::simulator {

// 시뮬레이션 로봇이 스냅샷/화면에 제공하는 상태. 로봇 모델은 WorldObject를
// 상속하고 이 상태를 구현한 뒤, 월드와 함께 Simulator에 주입한다.
struct RobotState {
  msgs::Twist command;
  std::map<std::string, double> joints;
  std::vector<float> scan_ranges;  // 진실 라이다 (잡음 없음)
  msgs::Pose2D scan_pose;
};

class SimRobot : public WorldObject {
 public:
  [[nodiscard]] virtual RobotState state() const = 0;
};

}  // namespace krosc::simulator
