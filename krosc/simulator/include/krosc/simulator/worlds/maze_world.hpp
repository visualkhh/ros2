#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/simulator/objects/puck.hpp"
#include "krosc/simulator/world.hpp"

namespace krosc::simulator {

// isaac rl_maze 트랙: 바깥 4.48 x 6.37 m 방 + 안쪽 벽. 시작할 때 퍽을 흩뿌린다.
class MazeWorld final : public World {
 public:
  explicit MazeWorld(int puck_count = 10, std::optional<std::uint32_t> seed = std::nullopt);

  [[nodiscard]] const msgs::Pose2D& start_pose() const { return start_pose_; }
  [[nodiscard]] const std::vector<Puck*>& pucks() const { return pucks_; }
  [[nodiscard]] Puck* graspable() const { return graspable_; }

  static constexpr std::array<double, 2> kCenter{0.0, 0.0};
  static constexpr std::array<double, 2> kSize{4.48, 6.37};

 private:
  [[nodiscard]] msgs::Pose2D free_spot(std::mt19937& rng, double radius) const;

  msgs::Pose2D start_pose_{0.0, 0.0, 0.0};
  std::vector<Puck*> pucks_;
  Puck* graspable_ = nullptr;
};

}  // namespace krosc::simulator
