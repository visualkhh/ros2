#include "krosc/simulator/objects/puck.hpp"

#include <utility>

namespace krosc::simulator {

Puck::Puck(PuckShape puck_shape, std::array<double, 3> puck_color) {
  radius = 0.06;
  height = 0.04;
  mass = 0.16;
  friction = 0.3;
  movable = true;
  sides = static_cast<int>(puck_shape);
  color = puck_color;
}

}  // namespace krosc::simulator
