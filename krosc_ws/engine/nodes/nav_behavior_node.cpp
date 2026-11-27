#include "nav_behavior_node.hpp"

#include <algorithm>
#include <any>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include "krosc/msgs.hpp"
#include "krosc/tf/math.hpp"

#include "llm_json.hpp"

namespace krosc_ws {

namespace {

std::string number(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return buffer;
}

std::string json_quoted(const std::string& value) {
  return "\"" + llm::escape_json(value) + "\"";
}

double wrap_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

}  // namespace

NavBehaviorNode::NavBehaviorNode(krosc::Context& context, std::string costmap_topic,
                                 std::string scan_topic)
    : Node(context, "nav_behavior"),
      costmap_topic_(std::move(costmap_topic)),
      scan_topic_(std::move(scan_topic)),
      nav_(std::make_unique<NavClient>(context,
                                       krosc::msgs::nav2::NAVIGATE_TO_POSE_ACTION)) {
  tf_listener_ = std::make_unique<krosc::TransformListener>(tf_buffer_, *this);
  grid_subscription_ = subscribe<krosc::msgs::OccupancyGrid>(
      costmap_topic_, [this](const krosc::msgs::OccupancyGrid& message) {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        grid_ = message;
        grid_seen_ = true;
      }, krosc::QOS_LATCHED);
  scan_subscription_ = subscribe<krosc::msgs::LaserScan>(
      scan_topic_, [this](const krosc::msgs::LaserScan& message) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        scan_ = message;
        scan_seen_ = true;
      }, krosc::QOS_SENSOR_DATA);
  create_service<krosc::msgs::String, krosc::msgs::String>(
      "nav/hide", [this](const krosc::msgs::String& request) {
        return handle_hide(request);
      });
  create_service<krosc::msgs::String, krosc::msgs::String>(
      "nav/approach", [this](const krosc::msgs::String& request) {
        return handle_approach(request);
      });
  create_service<krosc::msgs::TriggerRequest, krosc::msgs::TriggerResponse>(
      "nav/stop", [this](const krosc::msgs::TriggerRequest& request) {
        return handle_stop(request);
      });
  worker_ = std::thread([this] { worker_loop(); });
  logger().info("NavBehaviorNode: ready hide/approach costmap=%s scan=%s",
                costmap_topic_.c_str(), scan_topic_.c_str());
}

NavBehaviorNode::~NavBehaviorNode() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    pending_ = false;
  }
  condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  emergency_stop();
  shutdown();
}

void NavBehaviorNode::emergency_stop() {
  cancel_.store(true);
  publish<krosc::msgs::Twist>(
      "cmd_vel_nav",
      krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  std::lock_guard<std::mutex> lock(nav_mutex_);
  if (nav_ && !nav_goal_id_.empty()) {
    nav_->cancel(nav_goal_id_);
    nav_goal_id_.clear();
  }
}

krosc::msgs::String NavBehaviorNode::handle_hide(const krosc::msgs::String&) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_) {
      return krosc::msgs::String{
          R"({"accepted":false,"error":"nav behavior busy"})"};
    }
    cancel_.store(false);
    pending_ = true;
    pending_id_ = next_id_.fetch_add(1);
    pending_action_ = "hide";
    condition_.notify_one();
  }
  return krosc::msgs::String{"{\"accepted\":true,\"action\":\"hide\",\"request_id\":" +
                             std::to_string(pending_id_) + "}"};
}

krosc::msgs::String NavBehaviorNode::handle_approach(const krosc::msgs::String&) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_) {
      return krosc::msgs::String{
          R"({"accepted":false,"error":"nav behavior busy"})"};
    }
    cancel_.store(false);
    pending_ = true;
    pending_id_ = next_id_.fetch_add(1);
    pending_action_ = "approach";
    condition_.notify_one();
  }
  return krosc::msgs::String{
      "{\"accepted\":true,\"action\":\"approach\",\"request_id\":" +
      std::to_string(pending_id_) + "}"};
}

krosc::msgs::TriggerResponse NavBehaviorNode::handle_stop(
    const krosc::msgs::TriggerRequest&) {
  emergency_stop();
  return krosc::msgs::TriggerResponse{true, "nav stop"};
}

void NavBehaviorNode::worker_loop() {
  while (true) {
    std::uint64_t id = 0;
    std::string action;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return stopping_ || pending_; });
      if (stopping_) {
        return;
      }
      id = pending_id_;
      action = pending_action_;
      pending_ = false;
    }
    active_id_.store(id);
    std::string error;
    const bool ok = action == "approach" ? run_approach(error) : run_hide(error);
    if (!ok) {
      publish_status(id, "error", ",\"error\":" + json_quoted(error));
    } else {
      publish_status(id, "done", ",\"action\":" + json_quoted(action));
    }
    active_id_.store(0);
  }
}

bool NavBehaviorNode::hide_goal(double& goal_x, double& goal_y,
                                std::string& error) const {
  krosc::msgs::OccupancyGrid grid;
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    if (!grid_seen_ || grid_.data.empty() || grid_.info.width == 0 ||
        grid_.info.height == 0) {
      error = "costmap이 없다 (지도/SLAM 필요)";
      return false;
    }
    grid = grid_;
  }
  const std::string frame = grid.header.frame_id.empty() ? "map" : grid.header.frame_id;
  krosc::msgs::TransformStamped robot;
  try {
    robot = tf_buffer_.lookup_transform(frame, "base_link", std::nullopt, 0.5);
  } catch (const std::exception& exception) {
    error = "TF " + frame + "->base_link: " + exception.what();
    return false;
  }
  const double rx = robot.transform.translation.x;
  const double ry = robot.transform.translation.y;
  const double resolution = grid.info.resolution;
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  if (resolution <= 0.0) {
    error = "costmap resolution 오류";
    return false;
  }
  double best_distance = 0.0;
  bool have_blocked = false;
  int best_x = -1;
  int best_y = -1;
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  for (int cell_y = 0; cell_y < height; cell_y += 2) {
    for (int cell_x = 0; cell_x < width; cell_x += 2) {
      const int cost = grid.data[static_cast<std::size_t>(cell_y) * width + cell_x];
      if (cost < 0 || cost >= 50) {
        continue;
      }
      const double wx = origin_x + (cell_x + 0.5) * resolution;
      const double wy = origin_y + (cell_y + 0.5) * resolution;
      const double distance = std::hypot(wx - rx, wy - ry);
      if (distance < 1.0 || distance > 4.5) {
        continue;
      }
      const bool blocked = line_of_sight_blocked(grid, rx, ry, wx, wy);
      if (blocked) {
        if (!have_blocked || distance > best_distance) {
          have_blocked = true;
          best_distance = distance;
          best_x = cell_x;
          best_y = cell_y;
        }
      } else if (!have_blocked && distance > best_distance) {
        best_distance = distance;
        best_x = cell_x;
        best_y = cell_y;
      }
    }
  }
  if (best_x < 0) {
    error = "가려진 free cell을 찾지 못했다";
    return false;
  }
  goal_x = origin_x + (best_x + 0.5) * resolution;
  goal_y = origin_y + (best_y + 0.5) * resolution;
  return true;
}

bool NavBehaviorNode::line_of_sight_blocked(
    const krosc::msgs::OccupancyGrid& grid, double x0, double y0, double x1,
    double y1) const {
  const double resolution = grid.info.resolution;
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const double distance = std::hypot(x1 - x0, y1 - y0);
  const int steps = std::max(2, static_cast<int>(std::ceil(distance / (resolution * 0.5))));
  for (int index = 1; index < steps; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(steps);
    const double x = x0 + (x1 - x0) * ratio;
    const double y = y0 + (y1 - y0) * ratio;
    const int cell_x = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int cell_y = static_cast<int>(std::floor((y - origin_y) / resolution));
    if (cell_x < 0 || cell_y < 0 || cell_x >= width || cell_y >= height) {
      continue;
    }
    const int cost = grid.data[static_cast<std::size_t>(cell_y) * width + cell_x];
    if (cost < 0 || cost >= 65) {
      return true;
    }
  }
  return false;
}

bool NavBehaviorNode::run_hide(std::string& error) {
  double goal_x = 0.0;
  double goal_y = 0.0;
  if (!hide_goal(goal_x, goal_y, error)) {
    return false;
  }
  std::string frame = "map";
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    if (grid_seen_ && !grid_.header.frame_id.empty()) {
      frame = grid_.header.frame_id;
    }
  }
  krosc::msgs::nav2::NavigateToPoseGoal goal;
  goal.pose.header = krosc::msgs::Header{now(), frame};
  goal.pose.pose.position = {goal_x, goal_y, 0.0};
  goal.pose.pose.orientation = krosc::tfmath::from_yaw(0.0);
  const std::uint64_t id = active_id_.load();
  publish_status(id, "navigating",
                 ",\"goal\":[" + number(goal_x) + "," + number(goal_y) + "]");
  try {
    const auto handle = nav_->send_goal(
        goal, [this](const krosc::msgs::nav2::NavigateToPoseFeedback& feedback) {
          publish_status(active_id_.load(), "navigating",
                         ",\"remaining\":" + number(feedback.distance_remaining));
        });
    {
      std::lock_guard<std::mutex> lock(nav_mutex_);
      nav_goal_id_ = handle->goal_id();
    }
    const auto result = handle->result(90.0);
    {
      std::lock_guard<std::mutex> lock(nav_mutex_);
      if (nav_goal_id_ == handle->goal_id()) {
        nav_goal_id_.clear();
      }
    }
    if (result.status == krosc::GoalStatus::Succeeded) {
      return true;
    }
    if (result.result.has_value()) {
      try {
        const auto* nav_result =
            std::any_cast<krosc::msgs::nav2::NavigateToPoseResult>(&result.result);
        if (nav_result != nullptr && !nav_result->error_msg.empty()) {
          error = nav_result->error_msg;
        }
      } catch (const std::bad_any_cast&) {
      }
    }
    if (error.empty()) {
      error = "nav2 hide 실패";
    }
    return false;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

bool NavBehaviorNode::run_approach(std::string& error) {
  // 충돌 모니터가 footprint 때문에 0.22 m보다 먼저 멈출 수 있다. 정지 목표를
  // 조금 여유 있게 두고, 정체 시에도 이미 벽에 가까우면 성공으로 본다.
  constexpr double kStopDistance = 0.28;
  constexpr double kCloseEnough = 0.35;
  const auto started = std::chrono::steady_clock::now();
  auto last_progress = started;
  double last_distance = std::numeric_limits<double>::infinity();
  const std::uint64_t id = active_id_.load();
  publish_status(id, "approaching");
  while (!cancel_.load()) {
    krosc::msgs::LaserScan scan;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (!scan_seen_ || scan_.ranges.empty()) {
        error = "scan이 없다";
        return false;
      }
      scan = scan_;
    }
    double front = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      const double angle = scan.angle_min +
                           static_cast<double>(index) * scan.angle_increment;
      if (std::abs(wrap_angle(angle)) > 0.35) {
        continue;
      }
      const double range = scan.ranges[index];
      if (std::isfinite(range) && range >= scan.range_min &&
          range <= scan.range_max) {
        front = std::min(front, range);
      }
    }
    if (!std::isfinite(front)) {
      publish<krosc::msgs::Twist>(
          "cmd_vel_nav",
          krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
      error = "전방 0.35rad 안에 벽/물체가 없다";
      return false;
    }
    if (front <= kStopDistance) {
      break;
    }
    publish<krosc::msgs::Twist>(
        "cmd_vel_nav",
        krosc::msgs::Twist{{0.12, 0.0, 0.0}, {0.0, 0.0, 0.0}});
    publish_status(id, "approaching", ",\"front\":" + number(front));
    const auto current = std::chrono::steady_clock::now();
    if (front < last_distance - 0.01) {
      last_distance = front;
      last_progress = current;
    } else if (current - last_progress > std::chrono::seconds(3)) {
      publish<krosc::msgs::Twist>(
          "cmd_vel_nav",
          krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
      if (front <= kCloseEnough) {
        // 충돌 모니터/차체 때문에 더 못 가지만 벽에는 충분히 붙었다.
        return true;
      }
      error = "approach stalled (collision?)";
      return false;
    }
    if (current - started > std::chrono::seconds(20)) {
      publish<krosc::msgs::Twist>(
          "cmd_vel_nav",
          krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
      error = "approach timeout";
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  publish<krosc::msgs::Twist>(
      "cmd_vel_nav",
      krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  if (cancel_.load()) {
    error = "cancelled";
    return false;
  }
  return true;
}

void NavBehaviorNode::publish_status(std::uint64_t id, const std::string& stage,
                                     const std::string& extra_json) {
  std::string json = "{\"request_id\":" + std::to_string(id) + ",\"stage\":" +
                     json_quoted(stage);
  if (!extra_json.empty()) {
    json += extra_json;
  }
  json += "}";
  publish<krosc::msgs::String>("nav/status",
                               krosc::msgs::String{std::move(json)});
}

}  // namespace krosc_ws
