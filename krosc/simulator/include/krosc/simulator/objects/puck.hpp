#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "krosc/simulator/world.hpp"

namespace krosc::simulator {

enum class PuckShape : std::uint8_t { Circle = 32, Triangle = 3, Square = 4, Hexagon = 6 };

// 바닥에 놓인 작은 원기둥/각기둥 물체. World의 원 충돌을 그대로 쓴다.
class Puck final : public WorldObject {
 public:
  Puck(PuckShape shape, std::array<double, 3> color);
  [[nodiscard]] std::string kind() const override { return "puck"; }
};

}  // namespace krosc::simulator
