#include "krosc/nodes/nav2/costmap/costmap.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace krosc::nodes::nav2 {

namespace {
struct Kernel {
  int di = 0;
  int dj = 0;
  int value = 0;
};
}  // namespace

std::int8_t translate_cost(std::uint8_t cost) {
  if (cost == NO_INFORMATION) {
    return -1;
  }
  if (cost >= INSCRIBED_INFLATED_OBSTACLE) {
    return static_cast<std::int8_t>(cost - 154);
  }
  return cost == 0 ? 0 : static_cast<std::int8_t>(1 + (97 * (cost - 1)) / 251);
}

Costmap::Costmap(double resolution_value, int width_value, int height_value, double origin_x_value,
                 double origin_y_value)
    : resolution(resolution_value),
      width(width_value),
      height(height_value),
      origin_x(origin_x_value),
      origin_y(origin_y_value),
      costs(static_cast<std::size_t>(width_value) * static_cast<std::size_t>(height_value), FREE_SPACE) {}

std::optional<std::pair<int, int>> Costmap::world_to_map(double x, double y) const {
  if (!(resolution > 0.0)) {
    return std::nullopt;
  }
  const int i = static_cast<int>(std::floor((x - origin_x) / resolution));
  const int j = static_cast<int>(std::floor((y - origin_y) / resolution));
  if (i < 0 || j < 0 || i >= width || j >= height) {
    return std::nullopt;
  }
  return std::make_pair(i, j);
}

std::pair<double, double> Costmap::map_to_world(int i, int j) const {
  return {origin_x + (static_cast<double>(i) + 0.5) * resolution,
          origin_y + (static_cast<double>(j) + 0.5) * resolution};
}

std::uint8_t Costmap::cost_at(double x, double y) const {
  const auto cell = world_to_map(x, y);
  return cell ? cost(cell->first, cell->second) : NO_INFORMATION;
}

Costmap Costmap::from_occupancy_grid(const msgs::OccupancyGrid& grid, bool track_unknown_space) {
  const int width_value = static_cast<int>(grid.info.width);
  const int height_value = static_cast<int>(grid.info.height);
  Costmap map(grid.info.resolution, width_value, height_value, grid.info.origin.position.x,
              grid.info.origin.position.y);
  if (!map.valid()) {
    return map;
  }
  const std::uint8_t unknown = track_unknown_space ? NO_INFORMATION : FREE_SPACE;
  const std::size_t cells = map.costs.size();
  for (std::size_t index = 0; index < cells && index < grid.data.size(); ++index) {
    const std::int8_t value = grid.data[index];
    map.costs[index] = value < 0 ? unknown : (value >= LETHAL_THRESHOLD ? LETHAL_OBSTACLE : FREE_SPACE);
  }
  return map;
}

void Costmap::inflate(double inscribed_radius, double inflation_radius, double cost_scaling_factor) {
  if (!valid() || !(resolution > 0.0) || !(inflation_radius > 0.0)) {
    return;
  }
  const int reach = static_cast<int>(std::ceil(inflation_radius / resolution));
  std::vector<Kernel> kernel;
  for (int di = -reach; di <= reach; ++di) {
    for (int dj = -reach; dj <= reach; ++dj) {
      const double distance = std::hypot(static_cast<double>(di), static_cast<double>(dj)) * resolution;
      if (distance <= 0.0 || distance > inflation_radius) {
        continue;
      }
      const int value = distance <= inscribed_radius
                            ? static_cast<int>(INSCRIBED_INFLATED_OBSTACLE)
                            : static_cast<int>(252.0 * std::exp(-cost_scaling_factor * (distance - inscribed_radius)));
      if (value > 0) {
        kernel.push_back(Kernel{di, dj, value});
      }
    }
  }
  std::vector<std::pair<int, int>> lethal;
  std::vector<char> lethal_map(costs.size(), 0);
  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      if (cost(i, j) == LETHAL_OBSTACLE) {
        lethal.emplace_back(i, j);
        lethal_map[static_cast<std::size_t>(j) * width + i] = 1;
      }
    }
  }
  for (const auto& [i, j] : lethal) {
    bool edge = false;
    for (const auto& [di, dj] : {std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1}, std::pair{0, -1}}) {
      const int ni = i + di;
      const int nj = j + dj;
      if (ni < 0 || nj < 0 || ni >= width || nj >= height ||
          lethal_map[static_cast<std::size_t>(nj) * width + ni] == 0) {
        edge = true;
        break;
      }
    }
    if (!edge) {
      continue;
    }
    for (const auto& item : kernel) {
      const int x = i + item.di;
      const int y = j + item.dj;
      if (x < 0 || y < 0 || x >= width || y >= height) {
        continue;
      }
      const std::size_t index = static_cast<std::size_t>(y) * width + x;
      const std::uint8_t old = costs[index];
      const auto value = static_cast<std::uint8_t>(item.value);
      if (old == NO_INFORMATION) {
        if (value >= INSCRIBED_INFLATED_OBSTACLE) {
          costs[index] = value;
        }
      } else if (old < value) {
        costs[index] = value;
      }
    }
  }
}

msgs::OccupancyGrid Costmap::to_message(const msgs::Header& header) const {
  msgs::OccupancyGrid message;
  message.header = header;
  message.info.map_load_time = header.stamp;
  message.info.resolution = resolution;
  message.info.width = static_cast<std::uint32_t>(std::max(0, width));
  message.info.height = static_cast<std::uint32_t>(std::max(0, height));
  message.info.origin.position = {origin_x, origin_y, 0.0};
  message.info.origin.orientation = {};
  message.data.resize(costs.size());
  for (std::size_t index = 0; index < costs.size(); ++index) {
    message.data[index] = translate_cost(costs[index]);
  }
  return message;
}

}  // namespace krosc::nodes::nav2
