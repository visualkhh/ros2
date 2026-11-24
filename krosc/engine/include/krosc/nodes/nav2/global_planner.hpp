#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/node.hpp"

#include "krosc/nodes/nav2/costmap.hpp"

namespace krosc::nodes::nav2 {

struct Pose2 {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

using Cell = std::pair<int, int>;

struct Neighbor {
  int dx = 0;
  int dy = 0;
  double length = 1.0;
};

inline constexpr std::array<Neighbor, 8> kNeighbors{{
    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
    {1, 1, 1.4142135623730951}, {1, -1, 1.4142135623730951},
    {-1, 1, 1.4142135623730951}, {-1, -1, 1.4142135623730951},
}};

struct QueueEntry {
  double priority = 0.0;
  double cost = 0.0;
  Cell cell;
  bool operator>(const QueueEntry& other) const { return priority > other.priority; }
};

class PlannerError : public std::runtime_error {
 public:
  PlannerError(int error_code, std::string message) : std::runtime_error(std::move(message)), code(error_code) {}
  int code = msgs::nav2::ComputePathToPoseResult::UNKNOWN;
};

inline bool passable(int cost, bool allow_unknown) {
  return cost < INSCRIBED_INFLATED_OBSTACLE || (cost == NO_INFORMATION && allow_unknown);
}

inline std::pair<Cell, Cell> endpoints(const Costmap& costmap, const Pose2& start, const Pose2& goal,
                                       double tolerance, bool allow_unknown) {
  const auto s = costmap.world_to_map(start.x, start.y);
  const auto g = costmap.world_to_map(goal.x, goal.y);
  if (!s) {
    throw PlannerError(msgs::nav2::ComputePathToPoseResult::START_OUTSIDE_MAP, "시작이 지도 밖이다");
  }
  if (!g) {
    throw PlannerError(msgs::nav2::ComputePathToPoseResult::GOAL_OUTSIDE_MAP, "목표가 지도 밖이다");
  }
  if (costmap.cost(s->first, s->second) == LETHAL_OBSTACLE) {
    throw PlannerError(msgs::nav2::ComputePathToPoseResult::START_OCCUPIED, "시작이 벽 안이다");
  }
  Cell goal_cell = *g;
  if (!passable(costmap.cost(goal_cell.first, goal_cell.second), allow_unknown)) {
    const int reach = static_cast<int>(tolerance / costmap.resolution);
    std::optional<std::pair<double, Cell>> best;
    for (int di = -reach; di <= reach; ++di) {
      for (int dj = -reach; dj <= reach; ++dj) {
        if (std::hypot(static_cast<double>(di), static_cast<double>(dj)) > static_cast<double>(reach)) {
          continue;
        }
        const int i = g->first + di;
        const int j = g->second + dj;
        if (i < 0 || j < 0 || i >= costmap.width || j >= costmap.height) {
          continue;
        }
        if (!passable(costmap.cost(i, j), allow_unknown)) {
          continue;
        }
        const double distance = std::hypot(static_cast<double>(di), static_cast<double>(dj));
        if (!best || distance < best->first) {
          best = std::make_pair(distance, Cell{i, j});
        }
      }
    }
    if (!best) {
      throw PlannerError(msgs::nav2::ComputePathToPoseResult::GOAL_OCCUPIED, "목표가 벽이고 tolerance 안에 갈 곳이 없다");
    }
    goal_cell = best->second;
  }
  return {*s, goal_cell};
}

inline std::function<bool(int, int)> make_passable(const Costmap& costmap, Cell start, bool allow_unknown) {
  const double reach2 = std::pow(0.3 / costmap.resolution, 2.0);
  const bool stuck = !passable(costmap.cost(start.first, start.second), allow_unknown);
  return [&costmap, start, allow_unknown, reach2, stuck](int i, int j) {
    if (i < 0 || j < 0 || i >= costmap.width || j >= costmap.height) {
      return false;
    }
    const int cost = costmap.cost(i, j);
    if (passable(cost, allow_unknown)) {
      return true;
    }
    return stuck && cost != LETHAL_OBSTACLE &&
           std::pow(static_cast<double>(i - start.first), 2.0) +
                   std::pow(static_cast<double>(j - start.second), 2.0) <=
               reach2;
  };
}

inline double step_cost(const Costmap& costmap, Cell cell, double neutral_cost, double cost_factor) {
  const int cost = costmap.cost(cell.first, cell.second);
  return neutral_cost + cost_factor * (cost == NO_INFORMATION ? INSCRIBED_INFLATED_OBSTACLE - 1 : cost);
}

inline std::vector<std::pair<double, double>> to_world(const Costmap& costmap, const std::vector<Cell>& cells) {
  std::vector<std::pair<double, double>> result;
  result.reserve(cells.size());
  for (const auto& cell : cells) {
    result.push_back(costmap.map_to_world(cell.first, cell.second));
  }
  return result;
}

inline std::vector<Cell> walk_back(const std::map<Cell, Cell>& parents, Cell end) {
  std::vector<Cell> cells{end};
  while (true) {
    auto it = parents.find(cells.back());
    if (it == parents.end()) {
      break;
    }
    cells.push_back(it->second);
  }
  std::reverse(cells.begin(), cells.end());
  return cells;
}

inline std::vector<Cell> line_cells_bresenham(Cell a, Cell b) {
  int x = a.first;
  int y = a.second;
  const int x1 = b.first;
  const int y1 = b.second;
  const int dx = std::abs(x1 - x);
  const int dy = -std::abs(y1 - y);
  const int sx = x < x1 ? 1 : -1;
  const int sy = y < y1 ? 1 : -1;
  int error = dx + dy;
  std::vector<Cell> cells{{x, y}};
  while (x != x1 || y != y1) {
    const int e2 = 2 * error;
    if (e2 >= dy) {
      error += dy;
      x += sx;
    }
    if (e2 <= dx) {
      error += dx;
      y += sy;
    }
    cells.emplace_back(x, y);
  }
  return cells;
}

inline std::vector<std::pair<double, double>> densify(const Costmap& costmap, const std::vector<Cell>& corners) {
  if (corners.empty()) {
    return {};
  }
  std::vector<std::pair<double, double>> points{costmap.map_to_world(corners.front().first, corners.front().second)};
  for (std::size_t index = 0; index + 1 < corners.size(); ++index) {
    const auto a = costmap.map_to_world(corners[index].first, corners[index].second);
    const auto b = costmap.map_to_world(corners[index + 1].first, corners[index + 1].second);
    const int n = std::max(1, static_cast<int>(std::lround(std::hypot(b.first - a.first, b.second - a.second) / costmap.resolution)));
    for (int k = 1; k <= n; ++k) {
      points.emplace_back(a.first + (b.first - a.first) * k / n, a.second + (b.second - a.second) * k / n);
    }
  }
  return points;
}

class NavPlannerPlugin {
 public:
  virtual ~NavPlannerPlugin() = default;
  virtual void configure(Node& node, const std::string& name) {
    node_ = &node;
    name_ = name;
  }
  virtual std::vector<std::pair<double, double>> create_plan(const Costmap& costmap, const Pose2& start,
                                                              const Pose2& goal) = 0;
  [[nodiscard]] virtual std::string type() const = 0;

 protected:
  Node* node_ = nullptr;
  std::string name_;
};

using PlannerFactory = std::function<std::unique_ptr<NavPlannerPlugin>()>;

class PlannerRegistry {
 public:
  void add(const std::string& name, PlannerFactory factory) { factories_[name] = std::move(factory); }
  [[nodiscard]] std::unique_ptr<NavPlannerPlugin> create(const std::string& name) const {
    auto it = factories_.find(name);
    return it == factories_.end() ? nullptr : it->second();
  }
  [[nodiscard]] bool contains(const std::string& name) const { return factories_.count(name) != 0; }
  [[nodiscard]] std::vector<std::string> names() const {
    std::vector<std::string> result;
    for (const auto& [name, factory] : factories_) {
      (void)factory;
      result.push_back(name);
    }
    return result;
  }

 private:
  std::map<std::string, PlannerFactory> factories_;
};

// 플러그인 cpp에서 명시적으로 등록한다 (정적 라이브러리 링커가 registrar를 버리지 않게).
PlannerRegistry builtin_planners();

}  // namespace krosc::nodes::nav2
