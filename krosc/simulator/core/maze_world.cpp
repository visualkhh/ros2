#include "krosc/simulator/maze_world.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

namespace krosc::simulator {

namespace {

constexpr double kWallThickness = 0.05;
constexpr std::array<std::array<double, 4>, 20> kWalls{{
    {-2.24, -3.20, 2.24, -3.20},
    {2.24, -3.20, 2.24, 3.17},
    {2.24, 3.17, -2.24, 3.17},
    {-2.24, 3.17, -2.24, -3.20},
    {-1.19, -1.95, -0.41, -1.95},
    {0.35, -1.94, 1.89, -1.94},
    {-1.19, -1.20, -0.41, -1.20},
    {0.41, -1.16, 1.19, -1.16},
    {1.15, 0.33, 1.93, 0.33},
    {-1.95, 1.09, -1.17, 1.09},
    {1.11, 1.11, 1.89, 1.11},
    {-0.39, 1.81, 0.39, 1.81},
    {-1.18, 0.33, -1.18, 1.11},
    {-1.17, -2.71, -1.17, -1.93},
    {-1.17, -1.21, -1.17, -0.43},
    {-1.11, 1.69, -1.11, 2.49},
    {-0.43, -1.97, -0.43, -1.17},
    {0.37, 1.79, 0.37, 2.57},
    {1.15, 1.81, 1.15, 2.59},
    {1.17, -1.19, 1.17, 0.35},
}};

constexpr std::array<std::array<double, 3>, 10> kPuckColors{{
    {0.90, 0.20, 0.20}, {0.95, 0.55, 0.10}, {0.95, 0.85, 0.15}, {0.30, 0.75, 0.30}, {0.15, 0.70, 0.75},
    {0.25, 0.45, 0.90}, {0.60, 0.35, 0.85}, {0.90, 0.40, 0.70}, {0.95, 0.95, 0.95}, {0.55, 0.35, 0.20},
}};

}  // namespace

Puck::Puck(PuckShape puck_shape, std::array<double, 3> puck_color)
    : shape(puck_shape), color(puck_color) {
  radius = 0.06;
  height = 0.04;
  mass = 0.16;
  friction = 0.3;
  movable = true;
}

MazeWorld::MazeWorld(int puck_count, std::optional<std::uint32_t> seed) {
  for (const auto& line : kWalls) {
    const double cx = (line[0] + line[2]) / 2.0;
    const double cy = (line[1] + line[3]) / 2.0;
    add_wall(Wall{cx, cy, (std::abs(line[2] - line[0]) + kWallThickness) / 2.0,
                  (std::abs(line[3] - line[1]) + kWallThickness) / 2.0});
  }
  std::mt19937 rng(seed ? *seed : std::random_device{}());
  const auto shapes = std::array<PuckShape, 4>{PuckShape::Circle, PuckShape::Triangle, PuckShape::Square,
                                                PuckShape::Hexagon};
  for (int i = 0; i < puck_count; ++i) {
    auto puck = std::make_unique<Puck>(shapes[static_cast<std::size_t>(i) % shapes.size()],
                                      kPuckColors[static_cast<std::size_t>(i) % kPuckColors.size()]);
    auto* raw = static_cast<Puck*>(add(std::move(puck), free_spot(rng, 0.06)));
    pucks_.push_back(raw);
  }
}

msgs::Pose2D MazeWorld::free_spot(std::mt19937& rng, double radius) const {
  std::uniform_real_distribution<double> x_dist(kCenter[0] - kSize[0] / 2.0, kCenter[0] + kSize[0] / 2.0);
  std::uniform_real_distribution<double> y_dist(kCenter[1] - kSize[1] / 2.0, kCenter[1] + kSize[1] / 2.0);
  std::uniform_real_distribution<double> angle_dist(-3.14159265358979323846, 3.14159265358979323846);
  for (int attempt = 0; attempt < 1000; ++attempt) {
    const double x = x_dist(rng);
    const double y = y_dist(rng);
    const auto [px, py] = push_out(x, y, radius + 0.10);
    if (std::abs(px - x) > 1e-12 || std::abs(py - y) > 1e-12) {
      continue;
    }
    if (std::hypot(x - start_pose_.x, y - start_pose_.y) < 0.5) {
      continue;
    }
    bool too_close = false;
    for (const auto* puck : pucks_) {
      if (std::hypot(x - puck->pose.x, y - puck->pose.y) < 0.3) {
        too_close = true;
        break;
      }
    }
    if (too_close) {
      continue;
    }
    return {x, y, angle_dist(rng)};
  }
  throw std::runtime_error("퍽을 놓을 빈 곳을 찾지 못했다");
}

}  // namespace krosc::simulator
