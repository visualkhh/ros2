#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/sim/world.hpp"

namespace krosc::sim {

enum class PuckShape : std::uint8_t { Circle = 32, Triangle = 3, Square = 4, Hexagon = 6 };

class Puck final : public WorldObject {
 public:
  Puck(PuckShape shape, std::array<double, 3> color);
  [[nodiscard]] std::string kind() const override { return "puck"; }

  PuckShape shape;
  std::array<double, 3> color;
};

// isaac rl_maze 트랙: 바깥 4.48 x 6.37 m 방 + 안쪽 벽. 시작할 때 퍽을 흩뿌린다.
class MazeWorld final : public World {
 public:
  explicit MazeWorld(int puck_count = 10, std::optional<std::uint32_t> seed = std::nullopt);

  [[nodiscard]] const msgs::Pose2D& start_pose() const { return start_pose_; }
  [[nodiscard]] const std::vector<Puck*>& pucks() const { return pucks_; }

  static constexpr std::array<double, 2> kCenter{0.0, 0.0};
  static constexpr std::array<double, 2> kSize{4.48, 6.37};

 private:
  [[nodiscard]] msgs::Pose2D free_spot(std::mt19937& rng, double radius) const;

  msgs::Pose2D start_pose_{0.0, 0.0, 0.0};
  std::vector<Puck*> pucks_;
};

}  // namespace krosc::sim
