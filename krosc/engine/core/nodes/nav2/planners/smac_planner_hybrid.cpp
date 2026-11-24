#include "krosc/nodes/nav2/planners/smac_planner_hybrid.hpp"

#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <tuple>
#include <utility>

namespace krosc::nodes::nav2 {

namespace {

constexpr double kPi = 3.14159265358979323846;

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

void SmacPlannerHybrid::configure(Node& node, const std::string& name) {
  NavPlannerPlugin::configure(node, name);
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

}  // namespace krosc::nodes::nav2
