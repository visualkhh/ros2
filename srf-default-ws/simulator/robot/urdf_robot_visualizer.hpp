#pragma once

#include <map>
#include <string>
#include <vector>

#include "srf/simulator/robot.hpp"
#include "srf/urdf.hpp"

namespace srf_default_ws::sim {

namespace msgs = srf::msgs;

// URDF의 link visual을 읽어 현재 관절값으로 월드 좌표 조각을 만든다.
// 시뮬레이터 truth view가 손으로 그린 도형 대신 실제 URDF와 같은 형상을 쓰게 한다.
class UrdfRobotVisualizer {
 public:
  explicit UrdfRobotVisualizer(std::string urdf_text);

  [[nodiscard]] std::vector<srf::simulator::RobotVisual> visuals(
      const msgs::Pose2D& base, const std::map<std::string, double>& joints) const;

 private:
  srf::UrdfRobot robot_;
};

}  // namespace srf_default_ws::sim
