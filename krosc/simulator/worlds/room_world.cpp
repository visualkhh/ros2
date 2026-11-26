#include "krosc/simulator/worlds/room_world.hpp"

#include <algorithm>
#include <cmath>

namespace krosc::simulator {

RoomWorld::RoomWorld(double width, double height, double wall_height, double wall_thickness) {
  width = std::max(width, wall_thickness * 2.0 + 0.1);
  height = std::max(height, wall_thickness * 2.0 + 0.1);
  const double half_x = width / 2.0;
  const double half_y = height / 2.0;
  const double half_wall = wall_thickness / 2.0;
  add_wall(Wall{0.0, half_y + half_wall, half_x + wall_thickness, half_wall, wall_height});
  add_wall(Wall{0.0, -half_y - half_wall, half_x + wall_thickness, half_wall, wall_height});
  add_wall(Wall{-half_x - half_wall, 0.0, half_wall, half_y, wall_height});
  add_wall(Wall{half_x + half_wall, 0.0, half_wall, half_y, wall_height});
}

}  // namespace krosc::simulator
