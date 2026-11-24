#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"

namespace krosc::nodes::nav2 {

// nav2_costmap_2d의 Costmap2D 흉내: 칸마다 0~255 비용.
inline constexpr std::uint8_t FREE_SPACE = 0;
inline constexpr std::uint8_t INSCRIBED_INFLATED_OBSTACLE = 253;
inline constexpr std::uint8_t LETHAL_OBSTACLE = 254;
inline constexpr std::uint8_t NO_INFORMATION = 255;
inline constexpr int LETHAL_THRESHOLD = 65;  // OccupancyGrid 65% 이상 = 벽

// 비용 -> OccupancyGrid 값 (nav2의 cost_translation_table과 같은 규칙).
std::int8_t translate_cost(std::uint8_t cost);

class Costmap {
 public:
  Costmap() = default;
  Costmap(double resolution, int width, int height, double origin_x = 0.0, double origin_y = 0.0);

  [[nodiscard]] bool valid() const { return width > 0 && height > 0 && costs.size() == static_cast<std::size_t>(width) * height; }
  [[nodiscard]] std::optional<std::pair<int, int>> world_to_map(double x, double y) const;
  [[nodiscard]] std::pair<double, double> map_to_world(int i, int j) const;
  [[nodiscard]] std::uint8_t cost(int i, int j) const { return costs[static_cast<std::size_t>(j) * width + i]; }
  [[nodiscard]] std::uint8_t cost_at(double x, double y) const;
  void set_cost(int i, int j, std::uint8_t value) { costs[static_cast<std::size_t>(j) * width + i] = value; }

  [[nodiscard]] static Costmap from_occupancy_grid(const msgs::OccupancyGrid& grid, bool track_unknown_space = true);
  void inflate(double inscribed_radius, double inflation_radius, double cost_scaling_factor);
  [[nodiscard]] msgs::OccupancyGrid to_message(const msgs::Header& header) const;

  double resolution = 0.05;
  int width = 0;
  int height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<std::uint8_t> costs;
};

}  // namespace krosc::nodes::nav2
