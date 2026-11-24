#include "krosc/nodes/nav2/navfn_planner.hpp"

#include <cmath>
#include <map>
#include <queue>
#include <utility>

namespace krosc::nodes::nav2 {

void NavfnPlanner::configure(Node& node, const std::string& name) {
  NavPlannerPlugin::configure(node, name);
  tolerance_ = node.declare_parameter<double>(name + ".tolerance", 0.5, [](double value) { return value >= 0.0; });
  use_astar_ = node.declare_parameter<bool>(name + ".use_astar", false);
  allow_unknown_ = node.declare_parameter<bool>(name + ".allow_unknown", true);
}

std::vector<std::pair<double, double>> NavfnPlanner::create_plan(const Costmap& costmap, const Pose2& start,
                                                                 const Pose2& goal) {
  const auto [start_cell, goal_cell] = endpoints(costmap, start, goal, tolerance_.value(), allow_unknown_.value());
  const auto ok = make_passable(costmap, start_cell, allow_unknown_.value());
  const bool astar = use_astar_.value();
  const double neutral = 50.0;
  const double factor = 0.8;
  const auto heuristic = [&](Cell cell) {
    return astar ? std::hypot(static_cast<double>(cell.first - goal_cell.first),
                              static_cast<double>(cell.second - goal_cell.second)) *
                       neutral
                 : 0.0;
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
      return to_world(costmap, walk_back(parents, goal_cell));
    }
    for (const auto& neighbor : kNeighbors) {
      const Cell next{current.cell.first + neighbor.dx, current.cell.second + neighbor.dy};
      if (!ok(next.first, next.second)) {
        continue;
      }
      if (neighbor.dx != 0 && neighbor.dy != 0 &&
          (!ok(current.cell.first + neighbor.dx, current.cell.second) ||
           !ok(current.cell.first, current.cell.second + neighbor.dy))) {
        continue;
      }
      const double candidate = current.cost + neighbor.length * step_cost(costmap, next, neutral, factor);
      auto it = best.find(next);
      if (it == best.end() || candidate < it->second) {
        best[next] = candidate;
        parents[next] = current.cell;
        frontier.push({candidate + heuristic(next), candidate, next});
      }
    }
  }
  throw PlannerError(msgs::nav2::ComputePathToPoseResult::NO_VALID_PATH, "Navfn: 길이 없다");
}

}  // namespace krosc::nodes::nav2
