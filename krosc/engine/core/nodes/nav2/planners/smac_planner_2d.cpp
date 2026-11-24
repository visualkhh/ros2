#include "krosc/nodes/nav2/planners/smac_planner_2d.hpp"

#include <cmath>
#include <map>
#include <queue>
#include <utility>

namespace krosc::nodes::nav2 {

void SmacPlanner2D::configure(Node& node, const std::string& name) {
  NavPlannerPlugin::configure(node, name);
  tolerance_ = node.declare_parameter<double>(name + ".tolerance", 0.125,
                                              [](double value) { return value >= 0.0; });
  allow_unknown_ = node.declare_parameter<bool>(name + ".allow_unknown", true);
  cost_travel_multiplier_ = node.declare_parameter<double>(name + ".cost_travel_multiplier", 2.0,
                                                           [](double value) { return value >= 0.0; });
  max_iterations_ = node.declare_parameter<std::int64_t>(name + ".max_iterations", 1000000,
                                                          [](std::int64_t value) { return value > 0; });
  smooth_path_ = node.declare_parameter<bool>(name + ".smooth_path", true);
  w_smooth_ = node.declare_parameter<double>(name + ".smoother.w_smooth", 0.3,
                                             [](double value) { return value >= 0.0; });
  w_data_ = node.declare_parameter<double>(name + ".smoother.w_data", 0.2,
                                           [](double value) { return value >= 0.0; });
  smoother_iterations_ = node.declare_parameter<std::int64_t>(name + ".smoother.max_iterations", 1000,
                                                              [](std::int64_t value) { return value > 0; });
}

std::vector<std::pair<double, double>> SmacPlanner2D::create_plan(const Costmap& costmap, const Pose2& start,
                                                                  const Pose2& goal) {
  const auto [start_cell, goal_cell] = endpoints(costmap, start, goal, tolerance_.value(), allow_unknown_.value());
  const auto ok = make_passable(costmap, start_cell, allow_unknown_.value());
  const double penalty = cost_travel_multiplier_.value();
  const auto travel = [&](Cell cell) {
    const int cost = costmap.cost(cell.first, cell.second);
    return 1.0 + penalty * (cost == NO_INFORMATION ? INSCRIBED_INFLATED_OBSTACLE - 1 : cost) / 252.0;
  };
  std::map<Cell, double> best{{start_cell, 0.0}};
  std::map<Cell, Cell> parents;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> frontier;
  frontier.push({std::hypot(static_cast<double>(start_cell.first - goal_cell.first),
                            static_cast<double>(start_cell.second - goal_cell.second)),
                 0.0, start_cell});
  expanded_ = 0;
  const auto max_iterations = max_iterations_.value();
  while (!frontier.empty() && expanded_ < max_iterations) {
    const QueueEntry current = frontier.top();
    frontier.pop();
    auto best_it = best.find(current.cell);
    if (best_it == best.end() || current.cost > best_it->second) {
      continue;
    }
    ++expanded_;
    if (current.cell == goal_cell) {
      const auto points = to_world(costmap, walk_back(parents, goal_cell));
      return smooth_path_.value() ? smooth(costmap, points) : points;
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
      const double candidate = current.cost + neighbor.length * travel(next);
      auto it = best.find(next);
      if (it == best.end() || candidate < it->second) {
        best[next] = candidate;
        parents[next] = current.cell;
        frontier.push({candidate + std::hypot(static_cast<double>(next.first - goal_cell.first),
                                              static_cast<double>(next.second - goal_cell.second)),
                       candidate, next});
      }
    }
  }
  throw PlannerError(frontier.empty() ? msgs::nav2::ComputePathToPoseResult::NO_VALID_PATH
                                      : msgs::nav2::ComputePathToPoseResult::TIMEOUT,
                     "Smac2D: 길이 없다");
}

std::vector<std::pair<double, double>> SmacPlanner2D::smooth(
    const Costmap& costmap, const std::vector<std::pair<double, double>>& points) const {
  if (points.size() < 3) {
    return points;
  }
  std::vector<std::pair<double, double>> original = points;
  std::vector<std::pair<double, double>> result = points;
  const double w_smooth = w_smooth_.value();
  const double w_data = w_data_.value();
  for (std::int64_t iteration = 0; iteration < smoother_iterations_.value(); ++iteration) {
    double change = 0.0;
    for (std::size_t index = 1; index + 1 < result.size(); ++index) {
      const double x = result[index].first;
      const double y = result[index].second;
      const double nx = x + w_data * (original[index].first - x) +
                        w_smooth * (result[index - 1].first + result[index + 1].first - 2.0 * x);
      const double ny = y + w_data * (original[index].second - y) +
                        w_smooth * (result[index - 1].second + result[index + 1].second - 2.0 * y);
      if (costmap.cost_at(nx, ny) < INSCRIBED_INFLATED_OBSTACLE) {
        change += std::abs(nx - x) + std::abs(ny - y);
        result[index] = {nx, ny};
      }
    }
    if (change < 1e-6) {
      break;
    }
  }
  return result;
}

}  // namespace krosc::nodes::nav2
