#include "krosc/nodes/navigator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "krosc/math.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf_ros.hpp"

namespace krosc::nodes {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

std::int8_t occupancy_value(std::uint8_t cost) {
  if (cost == 255) {
    return -1;
  }
  if (cost == 254) {
    return 100;
  }
  if (cost == 253) {
    return 50;
  }
  return 0;
}

struct QueueEntry {
  float score = 0.0F;
  int cell = 0;
  bool operator>(const QueueEntry& other) const { return score > other.score; }
};

}  // namespace

Navigator::Navigator(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)) {
  const auto positive = [](const double& value) { return value > 0.0; };
  map_topic_ = declare_parameter<std::string>("map_topic", std::string("map"));
  goal_topic_ = declare_parameter<std::string>("goal_topic", std::string("goal_pose"));
  goals_topic_ = declare_parameter<std::string>("goals_topic", std::string("goal_poses"));
  preview_goal_topic_ = declare_parameter<std::string>("preview_goal_topic", std::string("preview_goal"));
  preview_goals_topic_ = declare_parameter<std::string>("preview_goals_topic", std::string("preview_goals"));
  cancel_topic_ = declare_parameter<std::string>("cancel_topic", std::string("cancel_goal"));
  plan_topic_ = declare_parameter<std::string>("plan_topic", std::string("plan"));
  status_topic_ = declare_parameter<std::string>("status_topic", std::string("goal_status"));
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", std::string("cmd_vel_nav"));
  global_costmap_topic_ = declare_parameter<std::string>("global_costmap_topic", std::string("global_costmap/costmap"));
  local_costmap_topic_ = declare_parameter<std::string>("local_costmap_topic", std::string("local_costmap/costmap"));
  map_frame_ = declare_parameter<std::string>("map_frame", std::string("map"));
  robot_frame_ = declare_parameter<std::string>("robot_frame", std::string("base_footprint"));
  controller_frequency_ = declare_parameter<double>("controller_frequency", 20.0, positive);
  vx_max_ = declare_parameter<double>("vx_max", 0.20, positive);
  wz_max_ = declare_parameter<double>("wz_max", 1.0, positive);
  lookahead_ = declare_parameter<double>("lookahead", 0.35, positive);
  inflation_ = declare_parameter<double>("inflation", 0.24, positive);
  xy_tolerance_ = declare_parameter<double>("xy_tolerance", 0.20, positive);
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.35, positive);
  local_costmap_size_ = declare_parameter<double>("local_costmap_size", 4.0, positive);
  costmap_period_ = declare_parameter<double>("costmap_period", 0.5, positive);

  listener_ = std::make_unique<TransformListener>(buffer_, *this);
  subscribe<msgs::OccupancyGrid>(map_topic_.value(), [this](const msgs::OccupancyGrid& map) { on_map(map); }, QOS_LATCHED);
  subscribe<msgs::PoseStamped>(goal_topic_.value(), [this](const msgs::PoseStamped& goal) { on_goal(goal); });
  subscribe<msgs::Path>(goals_topic_.value(), [this](const msgs::Path& goals) { on_goals(goals); });
  subscribe<msgs::PoseStamped>(preview_goal_topic_.value(),
                               [this](const msgs::PoseStamped& goal) { on_preview_goal(goal); });
  subscribe<msgs::Path>(preview_goals_topic_.value(),
                        [this](const msgs::Path& goals) { on_preview_goals(goals); });
  subscribe<msgs::Empty>(cancel_topic_.value(), [this](const msgs::Empty& message) { on_cancel(message); });
  create_timer(1.0 / controller_frequency_.value(), [this] { tick(); });
  logger().info("Navigator: topic=%s cmd=%s map=%s robot=%s", goal_topic_.value().c_str(),
                cmd_vel_topic_.value().c_str(), map_frame_.value().c_str(), robot_frame_.value().c_str());
}

bool Navigator::active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

std::size_t Navigator::path_size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return path_.size();
}

bool Navigator::build_grid(const msgs::OccupancyGrid& map) {
  if (map.info.width == 0 || map.info.height == 0 || !(map.info.resolution > 0.0) ||
      map.data.size() != static_cast<std::size_t>(map.info.width) * map.info.height) {
    return false;
  }
  Grid grid;
  grid.width = static_cast<int>(map.info.width);
  grid.height = static_cast<int>(map.info.height);
  grid.resolution = map.info.resolution;
  grid.origin_x = map.info.origin.position.x;
  grid.origin_y = map.info.origin.position.y;
  const std::size_t cells = static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
  grid.cost.assign(cells, 0);
  for (std::size_t index = 0; index < cells; ++index) {
    const std::int8_t value = map.data[index];
    if (value < 0) {
      grid.cost[index] = 255;  // 모르는 곳은 계획하지 않는다 (지도 안에서만 주행)
    } else if (value >= 50) {
      grid.cost[index] = 254;
    }
  }
  const int reach = static_cast<int>(std::ceil(inflation_.value() / grid.resolution));
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (grid.cost[static_cast<std::size_t>(y) * grid.width + x] != 254) {
        continue;
      }
      for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
          if (dx * dx + dy * dy > reach * reach) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height) {
            continue;
          }
          auto& cost = grid.cost[static_cast<std::size_t>(ny) * grid.width + nx];
          if (cost < 200) {
            cost = 253;
          }
        }
      }
    }
  }
  grid_ = std::move(grid);
  return true;
}

void Navigator::on_map(const msgs::OccupancyGrid& map) {
  std::lock_guard<std::mutex> lock(mutex_);
  path_.clear();
  waypoints_.clear();
  waypoint_indices_.clear();
  waypoint_index_ = 0;
  active_ = false;
  path_index_ = 0;
  if (map.info.width == 0 || map.info.height == 0) {
    grid_.reset();
    publish_empty_costmaps();
    publish_status("map unloaded");
    publish_command(0.0, 0.0);
    return;
  }
  if (!build_grid(map)) {
    grid_.reset();
    publish_empty_costmaps();
    publish_status("map invalid");
    publish_command(0.0, 0.0);
    return;
  }
  publish_global_costmap();
  publish_status("map ready");
}

std::optional<msgs::Pose2D> Navigator::robot_pose() const {
  try {
    const msgs::TransformStamped transform = buffer_.lookup_transform(map_frame_.value(), robot_frame_.value());
    return msgs::Pose2D{transform.transform.translation.x, transform.transform.translation.y,
                        tfmath::yaw_of(transform.transform.rotation)};
  } catch (const TransformException&) {
    return std::nullopt;
  }
}

std::optional<msgs::Pose2D> Navigator::to_map(const msgs::PoseStamped& pose) const {
  const double yaw = tfmath::yaw_of(pose.pose.orientation);
  if (pose.header.frame_id.empty() || pose.header.frame_id == map_frame_.value()) {
    return msgs::Pose2D{pose.pose.position.x, pose.pose.position.y, yaw};
  }
  try {
    const msgs::TransformStamped transform = buffer_.lookup_transform(map_frame_.value(), pose.header.frame_id);
    const double offset = tfmath::yaw_of(transform.transform.rotation);
    const double c = std::cos(offset);
    const double s = std::sin(offset);
    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;
    return msgs::Pose2D{transform.transform.translation.x + c * x - s * y,
                        transform.transform.translation.y + s * x + c * y, wrap(offset + yaw)};
  } catch (const TransformException&) {
    return std::nullopt;
  }
}

int Navigator::nearest_free(const Grid& grid, int cell) const {
  if (cell < 0 || cell >= grid.width * grid.height) {
    return -1;
  }
  if (grid.cost[static_cast<std::size_t>(cell)] < 200) {
    return cell;
  }
  const int x = cell % grid.width;
  const int y = cell / grid.width;
  for (int radius = 1; radius <= 24; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) {
          continue;
        }
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height) {
          continue;
        }
        const int index = ny * grid.width + nx;
        if (grid.cost[static_cast<std::size_t>(index)] < 200) {
          return index;
        }
      }
    }
  }
  return -1;
}

bool Navigator::plan(const Grid& grid, const msgs::Pose2D& start, const msgs::Pose2D& goal,
                     std::vector<msgs::Pose2D>& path) const {
  path.clear();
  if (!grid.valid()) {
    return false;
  }
  const auto cell_of = [&](const msgs::Pose2D& pose, int& x, int& y) {
    x = static_cast<int>(std::floor((pose.x - grid.origin_x) / grid.resolution));
    y = static_cast<int>(std::floor((pose.y - grid.origin_y) / grid.resolution));
    return x >= 0 && y >= 0 && x < grid.width && y < grid.height;
  };
  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;
  if (!cell_of(start, start_x, start_y) || !cell_of(goal, goal_x, goal_y)) {
    return false;
  }
  const int start_cell = nearest_free(grid, start_y * grid.width + start_x);
  const int goal_cell = nearest_free(grid, goal_y * grid.width + goal_x);
  if (start_cell < 0 || goal_cell < 0) {
    return false;
  }
  const int cells = grid.width * grid.height;
  std::vector<float> best(static_cast<std::size_t>(cells), std::numeric_limits<float>::infinity());
  std::vector<int> parent(static_cast<std::size_t>(cells), -1);
  const int gx = goal_cell % grid.width;
  const int gy = goal_cell / grid.width;
  const auto heuristic = [&](int cell) {
    const int x = cell % grid.width;
    const int y = cell / grid.width;
    return static_cast<float>(std::hypot(static_cast<double>(x - gx), static_cast<double>(y - gy)));
  };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  best[static_cast<std::size_t>(start_cell)] = 0.0F;
  open.push(QueueEntry{heuristic(start_cell), start_cell});
  static constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  bool found = start_cell == goal_cell;
  while (!open.empty() && !found) {
    const QueueEntry current = open.top();
    open.pop();
    if (current.score - heuristic(current.cell) > best[static_cast<std::size_t>(current.cell)] + 1e-4F) {
      continue;
    }
    const int x = current.cell % grid.width;
    const int y = current.cell / grid.width;
    for (int direction = 0; direction < 8; ++direction) {
      const int nx = x + kDx[direction];
      const int ny = y + kDy[direction];
      if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height) {
        continue;
      }
      const int index = ny * grid.width + nx;
      if (grid.cost[static_cast<std::size_t>(index)] >= 200) {
        continue;
      }
      if (direction >= 4) {  // 대각선은 모서리를 자르지 않게 옆 두 칸도 확인
        if (grid.cost[static_cast<std::size_t>(y * grid.width + nx)] >= 200 ||
            grid.cost[static_cast<std::size_t>(ny * grid.width + x)] >= 200) {
          continue;
        }
      }
      const float step = direction >= 4 ? 1.4142F : 1.0F;
      const float score = best[static_cast<std::size_t>(current.cell)] + step;
      if (score >= best[static_cast<std::size_t>(index)]) {
        continue;
      }
      best[static_cast<std::size_t>(index)] = score;
      parent[static_cast<std::size_t>(index)] = current.cell;
      open.push(QueueEntry{score + heuristic(index), index});
      if (index == goal_cell) {
        found = true;
      }
    }
  }
  if (!found) {
    return false;
  }
  std::vector<int> route;
  for (int cell = goal_cell; cell >= 0; cell = parent[static_cast<std::size_t>(cell)]) {
    route.push_back(cell);
    if (cell == start_cell) {
      break;
    }
  }
  std::reverse(route.begin(), route.end());
  path.push_back(start);
  for (const int cell : route) {
    const int x = cell % grid.width;
    const int y = cell / grid.width;
    const msgs::Pose2D point{grid.origin_x + (static_cast<double>(x) + 0.5) * grid.resolution,
                             grid.origin_y + (static_cast<double>(y) + 0.5) * grid.resolution, 0.0};
    if (std::hypot(point.x - path.back().x, point.y - path.back().y) < 1e-3) {
      continue;
    }
    path.push_back(point);
  }
  if (path.size() < 2) {
    path.push_back(goal);
  }
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    path[index].theta = std::atan2(path[index + 1].y - path[index].y, path[index + 1].x - path[index].x);
  }
  path.back().theta = goal.theta;
  if (path.size() > 2) {
    path[path.size() - 2].theta = path.back().theta;
  }
  return true;
}

void Navigator::publish_plan(const std::vector<msgs::Pose2D>& path) {
  msgs::Path message;
  message.header.stamp = now();
  message.header.frame_id = map_frame_.value();
  message.poses.reserve(path.size());
  for (const auto& point : path) {
    msgs::PoseStamped pose;
    pose.header = message.header;
    pose.pose.position = {point.x, point.y, 0.0};
    pose.pose.orientation = tfmath::from_yaw(point.theta);
    message.poses.push_back(std::move(pose));
  }
  publish(plan_topic_.value(), std::move(message), QOS_LATCHED);
}

void Navigator::publish_empty_costmaps() {
  msgs::OccupancyGrid empty;
  empty.header.stamp = now();
  empty.header.frame_id = map_frame_.value();
  publish(global_costmap_topic_.value(), empty, QOS_LATCHED);
  publish(local_costmap_topic_.value(), empty, QOS_LATCHED);
}

void Navigator::publish_global_costmap() {
  if (!grid_ || !grid_->valid()) {
    publish_empty_costmaps();
    return;
  }
  msgs::OccupancyGrid message;
  message.header.stamp = now();
  message.header.frame_id = map_frame_.value();
  message.info.resolution = grid_->resolution;
  message.info.width = static_cast<std::uint32_t>(grid_->width);
  message.info.height = static_cast<std::uint32_t>(grid_->height);
  message.info.origin.position = {grid_->origin_x, grid_->origin_y, 0.0};
  message.info.origin.orientation = tfmath::from_yaw(0.0);
  message.data.resize(grid_->cost.size());
  for (std::size_t index = 0; index < grid_->cost.size(); ++index) {
    message.data[index] = occupancy_value(grid_->cost[index]);
  }
  publish(global_costmap_topic_.value(), std::move(message), QOS_LATCHED);
}

void Navigator::publish_local_costmap() {
  if (!grid_ || !grid_->valid()) {
    return;
  }
  const auto pose = robot_pose();
  if (!pose) {
    return;
  }
  const double resolution = grid_->resolution;
  const int half = std::max(2, static_cast<int>(std::lround(local_costmap_size_.value() / resolution / 2.0)));
  const int width = half * 2;
  const int height = half * 2;
  const int center_x = static_cast<int>(std::floor((pose->x - grid_->origin_x) / resolution));
  const int center_y = static_cast<int>(std::floor((pose->y - grid_->origin_y) / resolution));
  const int start_x = center_x - half;
  const int start_y = center_y - half;
  msgs::OccupancyGrid message;
  message.header.stamp = now();
  message.header.frame_id = map_frame_.value();
  message.info.resolution = resolution;
  message.info.width = static_cast<std::uint32_t>(width);
  message.info.height = static_cast<std::uint32_t>(height);
  message.info.origin.position = {grid_->origin_x + static_cast<double>(start_x) * resolution,
                                  grid_->origin_y + static_cast<double>(start_y) * resolution, 0.0};
  message.info.origin.orientation = tfmath::from_yaw(0.0);
  message.data.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), -1);
  for (int y = 0; y < height; ++y) {
    const int grid_y = start_y + y;
    if (grid_y < 0 || grid_y >= grid_->height) {
      continue;
    }
    for (int x = 0; x < width; ++x) {
      const int grid_x = start_x + x;
      if (grid_x < 0 || grid_x >= grid_->width) {
        continue;
      }
      message.data[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)] =
          occupancy_value(grid_->cost[static_cast<std::size_t>(grid_y) * static_cast<std::size_t>(grid_->width) +
                                      static_cast<std::size_t>(grid_x)]);
    }
  }
  publish(local_costmap_topic_.value(), std::move(message), QOS_LATCHED);
}

void Navigator::publish_command(double linear, double angular) {
  msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  publish(cmd_vel_topic_.value(), std::move(twist), QOS_COMMAND);
}

void Navigator::publish_status(const std::string& text) {
  if (text == last_status_) {
    return;
  }
  last_status_ = text;
  publish(status_topic_.value(), msgs::String{text}, QOS_LATCHED);
  logger().info("navigator: %s", text.c_str());
}

void Navigator::stop(const std::string& status) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    path_.clear();
    waypoints_.clear();
    waypoint_indices_.clear();
    waypoint_index_ = 0;
    path_index_ = 0;
    publish_status(status);
  }
  publish_command(0.0, 0.0);
  publish_plan(std::vector<msgs::Pose2D>{});
}

void Navigator::handle_goal(const msgs::PoseStamped& goal_message, bool preview) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!grid_) {
    publish_status("no map (load a map first)");
    return;
  }
  const auto goal = to_map(goal_message);
  if (!goal) {
    publish_status("goal frame TF 없음");
    return;
  }
  const auto start = robot_pose();
  if (!start) {
    publish_status("no map pose (2D Pose Estimate first)");
    return;
  }
  std::vector<msgs::Pose2D> planned;
  if (!plan(*grid_, *start, *goal, planned)) {
    publish_status("plan failed (goal blocked?)");
    return;
  }
  path_ = planned;
  path_index_ = 0;
  waypoints_ = {*goal};
  waypoint_indices_ = {path_.size() - 1};
  waypoint_index_ = 0;
  active_ = !preview;
  publish_plan(path_);
  publish_status(preview ? "preview" : "following");
}

void Navigator::handle_goals(const msgs::Path& goals, bool preview) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (goals.poses.empty()) {
    path_.clear();
    waypoints_.clear();
    waypoint_indices_.clear();
    waypoint_index_ = 0;
    path_index_ = 0;
    if (!preview) {
      active_ = false;
      publish_command(0.0, 0.0);
    }
    publish_plan({});
    publish_status("no goals");
    return;
  }
  if (!grid_) {
    publish_status("no map (load a map first)");
    return;
  }
  const auto start = robot_pose();
  if (!start) {
    publish_status("no map pose (2D Pose Estimate first)");
    return;
  }
  std::vector<msgs::Pose2D> combined;
  std::vector<msgs::Pose2D> waypoints;
  std::vector<std::size_t> waypoint_indices;
  msgs::Pose2D current = *start;
  for (const auto& pose : goals.poses) {
    const auto goal = to_map(pose);
    if (!goal) {
      continue;
    }
    std::vector<msgs::Pose2D> segment;
    if (!plan(*grid_, current, *goal, segment)) {
      publish_status("plan failed (multi goal)");
      return;
    }
    if (!combined.empty() && !segment.empty()) {
      segment.erase(segment.begin());
    }
    combined.insert(combined.end(), segment.begin(), segment.end());
    waypoints.push_back(*goal);
    waypoint_indices.push_back(combined.size() - 1);
    current = *goal;
  }
  if (combined.size() < 2 || waypoints.empty()) {
    publish_status("no goals");
    return;
  }
  path_ = std::move(combined);
  waypoints_ = std::move(waypoints);
  waypoint_indices_ = std::move(waypoint_indices);
  waypoint_index_ = 0;
  path_index_ = 0;
  active_ = !preview;
  publish_plan(path_);
  publish_status(preview ? "preview" : "following");
}

void Navigator::on_goal(const msgs::PoseStamped& goal) { handle_goal(goal, false); }
void Navigator::on_goals(const msgs::Path& goals) { handle_goals(goals, false); }
void Navigator::on_preview_goal(const msgs::PoseStamped& goal) { handle_goal(goal, true); }
void Navigator::on_preview_goals(const msgs::Path& goals) { handle_goals(goals, true); }

void Navigator::on_cancel(const msgs::Empty& message) {
  (void)message;
  stop("canceled");
}

void Navigator::tick() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (now() - last_costmap_ >= costmap_period_.value()) {
      publish_local_costmap();
      last_costmap_ = now();
    }
  }

  double linear = 0.0;
  double angular = 0.0;
  bool publish = false;
  bool finished = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || path_.size() < 2 || waypoints_.empty() || waypoint_indices_.empty()) {
      return;
    }
    const auto pose = robot_pose();
    if (!pose) {
      publish_status("no map pose");
      return;
    }
    if (waypoint_index_ >= waypoints_.size()) {
      waypoint_index_ = waypoints_.size() - 1;
    }
    const msgs::Pose2D& waypoint = waypoints_[waypoint_index_];
    const std::size_t segment_end = std::min(waypoint_indices_[waypoint_index_], path_.size() - 1);
    const msgs::Pose2D& path_goal = path_[segment_end];
    const double goal_distance = std::hypot(path_goal.x - pose->x, path_goal.y - pose->y);

    if (goal_distance <= xy_tolerance_.value()) {
      const double yaw_error = wrap(waypoint.theta - pose->theta);
      if (std::abs(yaw_error) <= yaw_tolerance_.value()) {
        if (waypoint_index_ + 1 < waypoints_.size()) {
          ++waypoint_index_;
          linear = 0.0;
          angular = 0.0;
          publish = true;
          publish_status("waypoint reached");
        } else {
          active_ = false;
          path_.clear();
          waypoints_.clear();
          waypoint_indices_.clear();
          waypoint_index_ = 0;
          path_index_ = 0;
          publish_status("reached");
          finished = true;
        }
      } else {
        // 중간 웨이포인트의 방향까지 맞춘 뒤 다음 구간으로 넘어간다.
        linear = 0.0;
        angular = std::clamp(1.5 * yaw_error, -wz_max_.value(), wz_max_.value());
        publish = true;
        publish_status("aligning waypoint");
      }
    } else {
      std::size_t closest = std::min(path_index_, segment_end);
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t index = closest; index <= segment_end; ++index) {
        const double distance = std::hypot(path_[index].x - pose->x, path_[index].y - pose->y);
        if (distance < best) {
          best = distance;
          closest = index;
        }
      }
      path_index_ = closest;
      std::size_t target = segment_end;
      for (std::size_t index = closest; index <= segment_end; ++index) {
        if (std::hypot(path_[index].x - pose->x, path_[index].y - pose->y) >= lookahead_.value()) {
          target = index;
          break;
        }
      }
      const double dx = path_[target].x - pose->x;
      const double dy = path_[target].y - pose->y;
      const double desired = std::atan2(dy, dx);
      const double error = wrap(desired - pose->theta);
      if (std::abs(error) > 1.2) {
        linear = 0.0;
        angular = std::clamp(1.8 * error, -wz_max_.value(), wz_max_.value());
      } else {
        linear = vx_max_.value() * std::max(0.25, 1.0 - std::abs(error) / 1.6);
        angular = std::clamp(1.8 * error, -wz_max_.value(), wz_max_.value());
      }
      publish = true;
      publish_status("following");
    }
  }
  if (finished) {
    publish_command(0.0, 0.0);
    return;
  }
  if (publish) {
    publish_command(linear, angular);
  }
}

}  // namespace krosc::nodes
