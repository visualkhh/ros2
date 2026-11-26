#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/simulator/world.hpp"

namespace krosc::simulator {

// 시뮬레이션 로봇이 스냅샷/화면에 제공하는 상태. 로봇 모델은 WorldObject를
// 상속하고 이 상태를 구현한 뒤, 월드와 함께 Simulator에 주입한다.
enum class RobotVisualType { Box, Cylinder };

// 로봇 링크를 화면에 그리기 위한 3D 조각. 로봇 모델이 state()에서 채운다.
struct RobotVisual {
  RobotVisualType type = RobotVisualType::Box;
  msgs::Transform transform;
  std::array<double, 3> size{};   // Box: full x/y/z
  double radius = 0.0;            // Cylinder
  double length = 0.0;            // Cylinder
  int sides = 24;
  std::array<double, 3> color{0.55, 0.58, 0.65};
};

struct RobotState {
  msgs::Twist command;
  std::map<std::string, double> joints;
  std::vector<float> scan_ranges;  // 진실 라이다 (잡음 없음)
  msgs::Pose2D scan_pose;
  std::vector<RobotVisual> visuals;
};

class SimulatorRobot : public WorldObject {
 public:
  [[nodiscard]] virtual RobotState state() const = 0;
};

}  // namespace krosc::simulator
