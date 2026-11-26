#pragma once

#include <array>

#include "krosc/msgs.hpp"
#include "krosc/simulator/world.hpp"

namespace krosc::simulator {

// 가운데가 빈 단순 사각 방. 벽 두께/높이는 월드 좌표 기준이다.
class RoomWorld final : public World {
 public:
  explicit RoomWorld(double width = 4.0, double height = 4.0, double wall_height = 0.25,
                     double wall_thickness = 0.05);
};

}  // namespace krosc::simulator
