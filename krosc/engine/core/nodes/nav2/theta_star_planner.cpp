#include "krosc/nodes/nav2/theta_star_planner.hpp"

#include <cmath>
#include <map>
#include <optional>
#include <queue>
#include <utility>

namespace krosc::nodes::nav2 {

void ThetaStarPlanner::configure(Node& node, const std::string& name) {
  NavPlannerPlugin::configure(node, name);
  w_euc_cost_ = node.declare_parameter<double>(name + ".w_euc_cost", 1.0, [](double value) { return value >= 0.0; });
  w_traversal_cost_ = node.declare_parameter<double>(name + ".w_traversal_cost", 2.0,
                                                     [](double value) { return value >= 0.0; });
  allow_unknown_ = node.declare_parameter<bool>(name + ".allow_unknown", true);
  tolerance_ = node.declare_parameter<double>(name + ".tolerance", 0.5, [](double value) { return value >= 0.0; });
}

std::vector<std::pair<double, double>> ThetaStarPlanner::create_plan(const Costmap& costmap, const Pose2& start,
                                                                     const Pose2& goal) {
  const auto [start_cell, goal_cell] = endpoints(costmap, start, goal, tolerance_.value(), allow_unknown_.value());
  const auto ok = make_passable(costmap, start_cell, allow_unknown_.value());
  const double w_euc = w_euc_cost_.value();
  const double w_trav = w_traversal_cost_.value();
  const auto edge = [&](Cell a, Cell b) -> std::optional<double> {
    const auto cells = line_cells_bresenham(a, b);
    double sum = 0.0;
    for (const auto& cell : cells) {
      if (!ok(cell.first, cell.second)) {
        return std::nullopt;
      }
      const int cost = costmap.cost(cell.first, cell.second);
      sum += cost == NO_INFORMATION ? INSCRIBED_INFLATED_OBSTACLE - 1 : cost;
    }
    const double mean = sum / static_cast<double>(std::max<std::size_t>(1, cells.size()));
    return std::hypot(static_cast<double>(b.first - a.first), static_cast<double>(b.second - a.second)) *
           (w_euc + w_trav * mean / 252.0);
  };
  const auto heuristic = [&](Cell cell) {
    return w_euc * std::hypot(static_cast<double>(cell.first - goal_cell.first),
                              static_cast<double>(cell.second - goal_cell.second));
  };
  std::map<Cell, double> best{{start_cell, 0.0}};
  std::map<Cell, Cell> parents;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> frontier;
  frontier.push({heuristic(start_cell), 0.0, start_cell});
  expanded_ = 0;
  while (!frontier.empty()) {
    const QueueEntry current = frontier.top();
    frontier.pop();
    auto best_it = best.find(current.cell);
    if (best_it == best.end() || current.cost > best_it->second) {
      continue;
    }
    ++expanded_;
    if (current.cell == goal_cell) {
      return densify(costmap, walk_back(parents, goal_cell));
    }
    std::optional<Cell> grandparent;
    auto parent_it = parents.find(current.cell);
    if (parent_it != parents.end()) {
      grandparent = parent_it->second;
    }
    for (const auto& neighbor : kNeighbors) {
      const Cell next{current.cell.first + neighbor.dx, current.cell.second + neighbor.dy};
      if (!ok(next.first, next.second)) {
        continue;
      }
      Cell via = current.cell;
      std::optional<double> value;
      if (grandparent) {
        if (const auto through = edge(*grandparent, next)) {
          via = *grandparent;
          value = best[*grandparent] + *through;
        }
      }
      if (!value) {
        if (const auto step = edge(current.cell, next)) {
          value = current.cost + *step;
        }
      }
      if (!value) {
        continue;
      }
      auto it = best.find(next);
      if (it == best.end() || *value < it->second) {
        best[next] = *value;
        parents[next] = via;
        frontier.push({*value + heuristic(next), *value, next});
      }
    }
  }
  throw PlannerError(msgs::nav2::ComputePathToPoseResult::NO_VALID_PATH, "ThetaStar: 길이 없다");
}

}  // namespace krosc::nodes::nav2
