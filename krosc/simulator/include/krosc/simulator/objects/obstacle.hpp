#pragma once

#include <array>
#include <string>

#include "krosc/simulator/world.hpp"

namespace krosc::simulator {

// 월드에 놓는 간단한 정적 장애물. 충돌은 World의 원 판정을 쓴다.
class BoxObstacle final : public WorldObject {
 public:
  BoxObstacle(double radius, double height, std::array<double, 3> color);
  [[nodiscard]] std::string kind() const override { return "box"; }
};

class CylinderObstacle final : public WorldObject {
 public:
  CylinderObstacle(double radius, double height, std::array<double, 3> color);
  [[nodiscard]] std::string kind() const override { return "cylinder"; }
};

}  // namespace krosc::simulator
