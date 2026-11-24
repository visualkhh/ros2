#include "krosc/nodes/nav2/planner_plugins.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

namespace krosc::nodes::nav2 {

namespace {
constexpr double kPi = 3.14159265358979323846;

struct QueueEntry {
  double priority = 0.0;
  double cost = 0.0;
  Cell cell;
  bool operator>(const QueueEntry& other) const { return priority > other.priority; }
};

struct State {
  int i = 0;
  int j = 0;
  int yaw_bin = 0;
  bool operator<(const State& other) const {
    return std::tie(i, j, yaw_bin) < std::tie(other.i, other.j, other.yaw_bin);
  }
};

struct StateEntry {
  double priority = 0.0;
  double cost = 0.0;
  State state;
  bool operator>(const StateEntry& other) const { return priority > other.priority; }
};

Pose2 move_pose(const Pose2& pose, double length, double dtheta) {
  if (dtheta == 0.0) {
    return {pose.x + length * std::cos(pose.yaw), pose.y + length * std::sin(pose.yaw), pose.yaw};
  }
  const double radius = length / dtheta;
  const double new_yaw = pose.yaw + dtheta;
  return {pose.x + radius * (std::sin(new_yaw) - std::sin(pose.yaw)),
          pose.y - radius * (std::cos(new_yaw) - std::cos(pose.yaw)),
          std::atan2(std::sin(new_yaw), std::cos(new_yaw))};
}

std::map<Cell, double> distance_map(Cell goal,
                                    const std::function<bool(int, int)>& passable_cell) {
  std::map<Cell, double> distance{{goal, 0.0}};
  std::priority_queue<std::pair<double, Cell>, std::vector<std::pair<double, Cell>>, std::greater<>> frontier;
  frontier.push({0.0, goal});
  while (!frontier.empty()) {
    const auto [current_distance, cell] = frontier.top();
    frontier.pop();
    if (current_distance > distance[cell]) {
      continue;
    }
    for (const auto& neighbor : kNeighbors) {
      const Cell next{cell.first + neighbor.dx, cell.second + neighbor.dy};
      if (!passable_cell(next.first, next.second)) {
        continue;
      }
      const double candidate = current_distance + neighbor.length;
      auto it = distance.find(next);
      if (it == distance.end() || candidate < it->second) {
        distance[next] = candidate;
        frontier.push({candidate, next});
      }
    }
  }
  return distance;
}
}  // namespace

void NavfnPlanner::configure(Node& node, const std::string& name) {
  GlobalPlanner::configure(node, name);
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

void ThetaStarPlanner::configure(Node& node, const std::string& name) {
  GlobalPlanner::configure(node, name);
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

void SmacPlanner2D::configure(Node& node, const std::string& name) {
  GlobalPlanner::configure(node, name);
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

void SmacPlannerHybrid::configure(Node& node, const std::string& name) {
  GlobalPlanner::configure(node, name);
  motion_model_ = node.declare_parameter<std::string>(
      name + ".motion_model_for_search", "DUBIN",
      [](const std::string& value) { return value == "DUBIN" || value == "REEDS_SHEPP"; });
  minimum_turning_radius_ = node.declare_parameter<double>(name + ".minimum_turning_radius", 0.25,
                                                           [](double value) { return value > 0.0; });
  angle_bins_ = node.declare_parameter<std::int64_t>(name + ".angle_quantization_bins", 72,
                                                     [](std::int64_t value) { return value > 0; });
  cost_penalty_ = node.declare_parameter<double>(name + ".cost_penalty", 2.0,
                                                 [](double value) { return value >= 0.0; });
  non_straight_penalty_ = node.declare_parameter<double>(name + ".non_straight_penalty", 1.2,
                                                         [](double value) { return value >= 1.0; });
  reverse_penalty_ = node.declare_parameter<double>(name + ".reverse_penalty", 2.0,
                                                    [](double value) { return value >= 1.0; });
  tolerance_ = node.declare_parameter<double>(name + ".tolerance", 0.25,
                                              [](double value) { return value >= 0.0; });
  allow_unknown_ = node.declare_parameter<bool>(name + ".allow_unknown", true);
  max_iterations_ = node.declare_parameter<std::int64_t>(name + ".max_iterations", 200000,
                                                          [](std::int64_t value) { return value > 0; });
}

std::vector<std::pair<double, double>> SmacPlannerHybrid::create_plan(const Costmap& costmap, const Pose2& start,
                                                                      const Pose2& goal) {
  const auto [start_cell, goal_cell] = endpoints(costmap, start, goal, tolerance_.value(), allow_unknown_.value());
  const auto ok = make_passable(costmap, start_cell, allow_unknown_.value());
  const double resolution = costmap.resolution;
  const double step = std::sqrt(2.0) * resolution;
  const double turn = step / minimum_turning_radius_.value();
  const int bins = static_cast<int>(angle_bins_.value());
  const bool reeds = motion_model_.value() == "REEDS_SHEPP";
  std::vector<int> directions{1};
  if (reeds) {
    directions.push_back(-1);
  }
  const std::array<double, 3> turns{0.0, turn, -turn};
  const auto to_state = [&](double x, double y, double yaw) -> std::optional<State> {
    const auto cell = costmap.world_to_map(x, y);
    if (!cell) {
      return std::nullopt;
    }
    int bin = static_cast<int>(std::lround(yaw / (2.0 * kPi) * bins)) % bins;
    if (bin < 0) {
      bin += bins;
    }
    return State{cell->first, cell->second, bin};
  };
  const auto [goal_i, goal_j] = goal_cell;
  const auto goal_distance = distance_map(goal_cell, ok);
  std::map<State, Pose2> poses;
  std::map<State, double> best;
  std::map<State, State> parents;
  std::priority_queue<StateEntry, std::vector<StateEntry>, std::greater<>> frontier;
  const auto first_state = to_state(start.x, start.y, start.yaw);
  if (!first_state) {
    throw PlannerError(msgs::nav2::ComputePathToPoseResult::START_OUTSIDE_MAP, "SmacHybrid: 시작이 지도 밖");
  }
  poses[*first_state] = start;
  best[*first_state] = 0.0;
  const auto goal_heuristic = [&](double x, double y) {
    const auto cell = costmap.world_to_map(x, y);
    if (!cell) {
      return std::numeric_limits<double>::infinity();
    }
    const auto it = goal_distance.find(*cell);
    return std::max(std::hypot(x - goal.x, y - goal.y),
                    it == goal_distance.end() ? std::numeric_limits<double>::infinity()
                                               : it->second * resolution);
  };
  frontier.push({goal_heuristic(start.x, start.y), 0.0, *first_state});
  expanded_ = 0;
  while (!frontier.empty() && expanded_ < max_iterations_.value()) {
    const StateEntry current = frontier.top();
    frontier.pop();
    const auto best_it = best.find(current.state);
    if (best_it == best.end() || current.cost > best_it->second) {
      continue;
    }
    ++expanded_;
    const Pose2 pose = poses[current.state];
    if ((current.state.i == goal_i && current.state.j == goal_j) ||
        std::hypot(pose.x - goal.x, pose.y - goal.y) < resolution) {
      std::vector<State> states{current.state};
      while (parents.count(states.back()) != 0) {
        states.push_back(parents[states.back()]);
      }
      std::vector<std::pair<double, double>> points;
      points.reserve(states.size());
      for (auto it = states.rbegin(); it != states.rend(); ++it) {
        points.emplace_back(poses[*it].x, poses[*it].y);
      }
      return points;
    }
    for (int direction : directions) {
      for (double dtheta : turns) {
        const Pose2 next_pose = move_pose(pose, direction * step, direction * dtheta);
        const auto next_state = to_state(next_pose.x, next_pose.y, next_pose.yaw);
        if (!next_state || !ok(next_state->i, next_state->j)) {
          continue;
        }
        double weight = 1.0;
        const int cost = costmap.cost(next_state->i, next_state->j);
        weight += cost_penalty_.value() *
                  (cost == NO_INFORMATION ? INSCRIBED_INFLATED_OBSTACLE - 1 : cost) / 252.0;
        if (dtheta != 0.0) {
          weight *= non_straight_penalty_.value();
        }
        if (direction < 0) {
          weight *= reverse_penalty_.value();
        }
        const double candidate = current.cost + step * weight;
        const auto it = best.find(*next_state);
        if (it == best.end() || candidate < it->second) {
          best[*next_state] = candidate;
          parents[*next_state] = current.state;
          poses[*next_state] = next_pose;
          frontier.push({candidate + goal_heuristic(next_pose.x, next_pose.y), candidate, *next_state});
        }
      }
    }
  }
  throw PlannerError(frontier.empty() ? msgs::nav2::ComputePathToPoseResult::NO_VALID_PATH
                                      : msgs::nav2::ComputePathToPoseResult::TIMEOUT,
                     "SmacHybrid: 길이 없다");
}

PlannerRegistry builtin_planners() {
  PlannerRegistry registry;
  registry.add("nav2_navfn_planner::NavfnPlanner", [] { return std::make_unique<NavfnPlanner>(); });
  registry.add("nav2_theta_star_planner::ThetaStarPlanner", [] { return std::make_unique<ThetaStarPlanner>(); });
  registry.add("nav2_smac_planner::SmacPlanner2D", [] { return std::make_unique<SmacPlanner2D>(); });
  registry.add("nav2_smac_planner::SmacPlannerHybrid", [] { return std::make_unique<SmacPlannerHybrid>(); });
  return registry;
}

}  // namespace krosc::nodes::nav2
