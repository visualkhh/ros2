#include "krosc/simulator/objects/obstacle.hpp"

#include <utility>

namespace krosc::simulator {

BoxObstacle::BoxObstacle(double obstacle_radius, double obstacle_height,
                         std::array<double, 3> obstacle_color) {
  radius = obstacle_radius;
  height = obstacle_height;
  mass = 10.0;
  friction = 0.5;
  is_static = true;
  sides = 4;
  color = obstacle_color;
}

CylinderObstacle::CylinderObstacle(double obstacle_radius, double obstacle_height,
                                   std::array<double, 3> obstacle_color) {
  radius = obstacle_radius;
  height = obstacle_height;
  mass = 10.0;
  friction = 0.5;
  is_static = true;
  sides = 24;
  color = obstacle_color;
}

}  // namespace krosc::simulator
