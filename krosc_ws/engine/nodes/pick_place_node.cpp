#include "pick_place_node.hpp"

#include <algorithm>
#include <any>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <utility>

#include "krosc/control/controller_manager.hpp"
#include "krosc/msgs.hpp"
#include "krosc/tf/math.hpp"

#include "llm_json.hpp"

namespace krosc_ws {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kOpenGrip = 0.019;   // 그리퍼 최대 개방
constexpr double kPuckGrip = 0.008;   // 4cm 퍽 기본 close (검출 폭이 없을 때)
constexpr double kGripperBaseGap = 0.0244;  // finger gap at grip=0
constexpr double kMaxObjectWidth = 0.072;   // detector가 발행하는 최대 폭

// 검출 폭에 맞춰 닫힘 위치를 계산한다. 물체보다 2mm 작게 목표를 두어
// 살짝 조이는 힘이 걸리게 한다. 4cm 퍽이면 기존 8mm와 거의 같다.
double close_grip_for(double width) {
  // open(0.019)과 같은 값이 되면 그리퍼가 실제로 닫히지 않는다.
  // 큰 물체도 최소 1.5mm는 조이도록 close 상한을 낮춘다.
  return std::clamp((width - 0.002 - kGripperBaseGap) / 2.0, 0.006, 0.0175);
}

std::string number(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return buffer;
}

std::string json_quoted(const std::string& value) {
  return "\"" + llm::escape_json(value) + "\"";
}

std::string lower_copy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return text;
}

double wrap_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

}  // namespace

PickPlaceNode::PickPlaceNode(krosc::Context& context, std::string objects_topic,
                             std::string info_topic)
    : Node(context, "pick_place"),
      objects_topic_(std::move(objects_topic)),
      info_topic_(std::move(info_topic)),
      move_(std::make_unique<MoveClient>(context,
                                         krosc::msgs::moveit::MOVE_ACTION)),
      nav_(std::make_unique<NavClient>(context,
                                       krosc::msgs::nav2::NAVIGATE_TO_POSE_ACTION)) {
  objects_subscription_ = subscribe<krosc::msgs::Path>(
      objects_topic_, [this](const krosc::msgs::Path& message) {
        std::lock_guard<std::mutex> lock(object_mutex_);
        objects_ = message;
        objects_seen_ = true;
      });
  info_subscription_ = subscribe<krosc::msgs::String>(
      info_topic_, [this](const krosc::msgs::String& message) {
        std::lock_guard<std::mutex> lock(info_mutex_);
        if (const auto width = llm::number_field(message.data, "width")) {
          object_width_ = *width;
        }
        if (const auto height = llm::number_field(message.data, "height")) {
          object_height_ = *height;
        }
        if (const auto color = llm::string_field(message.data, "color")) {
          object_color_ = *color;
        }
      });
  odom_subscription_ = subscribe<krosc::msgs::Odometry>(
      "odom", [this](const krosc::msgs::Odometry& message) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_x_ = message.pose.pose.position.x;
        odom_y_ = message.pose.pose.position.y;
        odom_yaw_ = krosc::tfmath::yaw_of(message.pose.pose.orientation);
        odom_seen_ = true;
      });
  create_service<krosc::msgs::String, krosc::msgs::String>(
      "pick/command", [this](const krosc::msgs::String& request) {
        return handle_command(request);
      });
  create_service<krosc::msgs::TriggerRequest, krosc::msgs::TriggerResponse>(
      "pick/stop", [this](const krosc::msgs::TriggerRequest&) {
        emergency_stop();
        return krosc::msgs::TriggerResponse{true, "pick stop"};
      });
  worker_ = std::thread([this] { worker_loop(); });
  logger().info("PickPlaceNode: ready service=pick/command objects=%s info=%s",
                objects_topic_.c_str(), info_topic_.c_str());
}

PickPlaceNode::~PickPlaceNode() {
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

void PickPlaceNode::emergency_stop() {
  cancel_.store(true);
  publish<krosc::msgs::Twist>(
      "cmd_vel_nav",
      krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    if (nav_ && !nav_goal_id_.empty()) {
      nav_->cancel(nav_goal_id_);
      nav_goal_id_.clear();
    }
  }
  std::lock_guard<std::mutex> lock(move_mutex_);
  if (move_ && !move_goal_id_.empty()) {
    move_->cancel(move_goal_id_);
    move_goal_id_.clear();
  }
}

krosc::msgs::String PickPlaceNode::handle_command(
    const krosc::msgs::String& request) {
  std::string action = llm::string_field(request.data, "action").value_or("pick");
  std::string color = llm::string_field(request.data, "color").value_or("yellow");
  std::string target = llm::string_field(request.data, "target").value_or(
      action == "place" ? "floor" : color);
  if (action != "pick" && action != "place") {
    return krosc::msgs::String{
        R"({"accepted":false,"error":"action은 pick|place"})"};
  }
  if (action == "pick") {
    std::lock_guard<std::mutex> object_lock(object_mutex_);
    if (!objects_seen_ || objects_.poses.empty()) {
      return krosc::msgs::String{
          R"({"accepted":false,"error":"sim_graspable 물체가 없다"})"};
    }
    const auto& position = objects_.poses.front().pose.position;
    target_x_ = position.x;
    target_y_ = position.y;
    target_z_ = position.z;
    have_target_ = true;
    std::lock_guard<std::mutex> info_lock(info_mutex_);
    target_width_ = object_width_;
    target_color_ = object_color_;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_) {
      return krosc::msgs::String{
          R"({"accepted":false,"error":"pick/place busy"})"};
    }
    cancel_.store(false);
    pending_ = true;
    pending_id_ = next_id_.fetch_add(1);
    pending_action_ = action;
    pending_color_ = std::move(color);
    pending_target_ = std::move(target);
    condition_.notify_one();
  }
  return krosc::msgs::String{"{\"accepted\":true,\"request_id\":" +
                             std::to_string(pending_id_) + "}"};
}

void PickPlaceNode::worker_loop() {
  while (true) {
    std::uint64_t id = 0;
    std::string action;
    std::string color;
    std::string target;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return stopping_ || pending_; });
      if (stopping_) {
        return;
      }
      id = pending_id_;
      action = pending_action_;
      color = pending_color_;
      target = pending_target_;
      pending_ = false;
    }
    std::string error;
    publish_status(id, "switching");
    if (!switch_arm(error)) {
      publish_status(id, "error",
                     ",\"error\":" + json_quoted("controller switch: " + error));
      continue;
    }
    publish_status(id, "executing",
                   ",\"action\":" + json_quoted(action) +
                       ",\"target\":" + json_quoted(target));
    const bool ok = action == "place" ? place(id, target, error) : pick(id, color, error);
    if (!ok) {
      publish_status(id, "error", ",\"error\":" + json_quoted(error));
      continue;
    }
    publish_status(id, "done", ",\"action\":" + json_quoted(action));
  }
}

bool PickPlaceNode::switch_arm(std::string& error) {
  krosc::control::SwitchControllerRequest request;
  request.start_controllers = {"joint_trajectory_controller"};
  request.stop_controllers = {"arm_command_controller"};
  request.strictness = 0;
  try {
    const auto response = context().services().call<
        krosc::control::SwitchControllerRequest,
        krosc::control::SwitchControllerResponse>(
        "controller_manager/switch_controller", request);
    if (!response.ok) {
      error = response.message;
      return false;
    }
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

bool PickPlaceNode::move_to(std::uint64_t id, const std::string& stage,
                            double x, double y, double z, double pitch_deg,
                            double grip_m, std::string& error) {
  krosc::msgs::moveit::MoveGroupGoal goal;
  goal.target_pose.header = krosc::msgs::Header{now(), "base_link"};
  goal.target_pose.pose.position = {x, y, z};
  const double yaw = std::atan2(y, x);
  goal.target_pose.pose.orientation =
      krosc::tfmath::from_rpy(0.0, -pitch_deg * kPi / 180.0, yaw);
  goal.cartesian = false;  // joint-space: 카메라/차체에 막히는 직선 계획을 피한다
  goal.plan_only = false;
  goal.grip = grip_m;
  try {
    const auto handle = move_->send_goal(
        goal, [this, id, stage](const krosc::msgs::moveit::MoveGroupFeedback& feedback) {
          publish_status(id, stage,
                         ",\"progress\":" + number(feedback.progress) +
                             ",\"state\":" + json_quoted(feedback.state));
        });
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      move_goal_id_ = handle->goal_id();
    }
    const auto result = handle->result(45.0);
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      if (move_goal_id_ == handle->goal_id()) {
        move_goal_id_.clear();
      }
    }
    if (result.status == krosc::GoalStatus::Succeeded) {
      if (result.result.has_value()) {
        try {
          const auto* move_result =
              std::any_cast<krosc::msgs::moveit::MoveGroupResult>(&result.result);
          if (move_result != nullptr &&
              move_result->error_code != krosc::msgs::moveit::MoveGroupResult::NONE) {
            error = move_result->error_msg.empty() ? "MoveIt 계획/실행 실패"
                                                   : move_result->error_msg;
            return false;
          }
        } catch (const std::bad_any_cast&) {
        }
      }
      return true;
    }
    if (result.result.has_value()) {
      try {
        const auto* move_result =
            std::any_cast<krosc::msgs::moveit::MoveGroupResult>(&result.result);
        if (move_result != nullptr && !move_result->error_msg.empty()) {
          error = move_result->error_msg;
        }
      } catch (const std::bad_any_cast&) {
      }
    }
    if (error.empty()) {
      error = "MoveIt " + stage + " 실패";
    }
    return false;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

bool PickPlaceNode::object_in_base(double& x, double& y, double& z, double& yaw,
                                   std::string& error) const {
  double target_x = 0.0;
  double target_y = 0.0;
  double target_z = 0.0;
  {
    std::lock_guard<std::mutex> lock(object_mutex_);
    if (have_target_) {
      target_x = target_x_;
      target_y = target_y_;
      target_z = target_z_;
    } else if (objects_seen_ && !objects_.poses.empty()) {
      const auto& position = objects_.poses.front().pose.position;
      target_x = position.x;
      target_y = position.y;
      target_z = position.z;
    } else {
      error = "sim_graspable 물체가 없다";
      return false;
    }
  }
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!odom_seen_) {
      error = "odom이 없다";
      return false;
    }
    robot_x = odom_x_;
    robot_y = odom_y_;
    robot_yaw = odom_yaw_;
  }
  const double dx = target_x - robot_x;
  const double dy = target_y - robot_y;
  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);
  x = c * dx + s * dy;
  y = -s * dx + c * dy;
  z = target_z;
  yaw = std::atan2(y, x);
  return true;
}

bool PickPlaceNode::object_odom(double& x, double& y, double& z,
                                std::string& error) const {
  std::lock_guard<std::mutex> lock(object_mutex_);
  if (have_target_) {
    x = target_x_;
    y = target_y_;
    z = target_z_;
    return true;
  }
  if (!objects_seen_ || objects_.poses.empty()) {
    error = "sim_graspable 물체가 없다";
    return false;
  }
  const auto& position = objects_.poses.front().pose.position;
  x = position.x;
  y = position.y;
  z = position.z;
  return true;
}

bool PickPlaceNode::approach(std::uint64_t id, std::string& error) {
  // 팔 base(-0.092m)에서 0.25m 앞이면 3D 도달거리에 여유가 있다.
  constexpr double kStandoff = 0.25;
  const auto started = std::chrono::steady_clock::now();
  auto last_progress = started;
  double last_range = std::numeric_limits<double>::infinity();
  {
    double ox = 0.0;
    double oy = 0.0;
    double oz = 0.0;
    if (!object_odom(ox, oy, oz, error)) {
      return false;
    }
    (void)oz;
    double rx = 0.0;
    double ry = 0.0;
    double ryaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!odom_seen_) {
        error = "odom이 없다";
        return false;
      }
      rx = odom_x_;
      ry = odom_y_;
      ryaw = odom_yaw_;
    }
    (void)ryaw;
    // 아주 먼 물체만 nav2로 보낸다. 0.45m 정도 앞 물체는 지도 없이
    // 로컬 직진 approach로 충분하고, 실기에서 지도 미로드 상태에도 동작한다.
    if (std::hypot(ox - rx, oy - ry) > 0.80) {
      if (!navigate_to_puck(id, error)) {
        return false;
      }
    }
  }
  publish_status(id, "approach");
  while (!cancel_.load()) {
    double ox = 0.0;
    double oy = 0.0;
    double oz = 0.0;
    if (!object_odom(ox, oy, oz, error)) {
      return false;
    }
    (void)oz;
    double rx = 0.0;
    double ry = 0.0;
    double ryaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!odom_seen_) {
        error = "odom이 없다";
        return false;
      }
      rx = odom_x_;
      ry = odom_y_;
      ryaw = odom_yaw_;
    }
    const double dx = ox - rx;
    const double dy = oy - ry;
    const double range = std::hypot(dx, dy);
    const double bearing = wrap_angle(std::atan2(dy, dx) - ryaw);
    // IK plan_only로 확인한 도달 한계가 x≈0.25m다. dead zone 없이 정확히
    // standoff에서 멈춘다.
    if (range <= kStandoff) {
      break;
    }
    const double angular = std::clamp(1.6 * bearing, -1.0, 1.0);
    const double linear =
        (std::abs(bearing) < 0.35 && range > kStandoff) ? 0.10 : 0.0;
    publish<krosc::msgs::Twist>(
        "cmd_vel_nav",
        krosc::msgs::Twist{{linear, 0.0, 0.0}, {0.0, 0.0, angular}});
    const auto current = std::chrono::steady_clock::now();
    if (range < last_range - 0.01) {
      last_range = range;
      last_progress = current;
    } else if (current - last_progress > std::chrono::seconds(3)) {
      publish<krosc::msgs::Twist>(
          "cmd_vel_nav",
          krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
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

bool PickPlaceNode::navigate_to_puck(std::uint64_t id, std::string& error) {
  double ox = 0.0;
  double oy = 0.0;
  double oz = 0.0;
  if (!object_odom(ox, oy, oz, error)) {
    return false;
  }
  (void)oz;
  double rx = 0.0;
  double ry = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!odom_seen_) {
      error = "odom이 없다";
      return false;
    }
    rx = odom_x_;
    ry = odom_y_;
  }
  const double range = std::hypot(ox - rx, oy - ry);
  constexpr double kStandoff = 0.34;
  if (range <= kStandoff + 0.05) {
    return true;
  }
  const double goal_x = ox + (rx - ox) / range * kStandoff;
  const double goal_y = oy + (ry - oy) / range * kStandoff;
  krosc::msgs::nav2::NavigateToPoseGoal goal;
  goal.pose.header = krosc::msgs::Header{now(), "odom"};
  goal.pose.pose.position = {goal_x, goal_y, 0.0};
  goal.pose.pose.orientation =
      krosc::tfmath::from_yaw(std::atan2(oy - goal_y, ox - goal_x));
  publish_status(id, "navigating",
                 ",\"goal\":[" + number(goal_x) + "," + number(goal_y) + "]");
  try {
    const auto handle = nav_->send_goal(
        goal, [this, id](const krosc::msgs::nav2::NavigateToPoseFeedback& feedback) {
          publish_status(id, "navigating",
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
        const auto* nav_result = std::any_cast<krosc::msgs::nav2::NavigateToPoseResult>(
            &result.result);
        if (nav_result != nullptr && !nav_result->error_msg.empty()) {
          error = nav_result->error_msg;
        }
      } catch (const std::bad_any_cast&) {
      }
    }
    if (error.empty()) {
      error = "nav2 접근 실패 (지도/SLAM 확인)";
    }
    return false;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

bool PickPlaceNode::pick(std::uint64_t id, const std::string& color,
                         std::string& error) {
  if (!approach(id, error)) {
    return false;
  }
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  if (!object_in_base(x, y, z, yaw, error)) {
    return false;
  }
  if (x < 0.12 || x > 0.40 || std::abs(y) > 0.22) {
    error = "물체가 팔 작업영역 밖이다 (base x=" + number(x) +
            " y=" + number(y) + ")";
    return false;
  }

  double width = 0.0;
  std::string detected_color;
  bool latched = false;
  {
    std::lock_guard<std::mutex> object_lock(object_mutex_);
    latched = have_target_;
    if (latched) {
      width = target_width_;
      detected_color = target_color_;
    }
  }
  if (!latched) {
    std::lock_guard<std::mutex> lock(info_mutex_);
    width = object_width_;
    detected_color = object_color_;
  }
  if (width > kMaxObjectWidth) {
    error = "물체 폭 " + number(width) + "m가 허용 폭(" +
            number(kMaxObjectWidth) + "m)보다 크다";
    return false;
  }
  const double close_grip = width > 0.030 ? close_grip_for(width) : kPuckGrip;
  logger().info("pick: color=%s detected=%s width=%.3f open=%.3f close=%.3f target=(%.3f, %.3f, %.3f)",
                color.c_str(), detected_color.c_str(), width, kOpenGrip, close_grip,
                x, y, z);

  const double center = z;
  // 팔을 수평으로 뻗으면 IK가 부족하다. approach부터 아래로 굽힌 자세로
  // 물체 바로 위(center+0.07)에 올리고, descend에서 45°로 내려 잡는다.
  if (!move_to(id, "approach", x, y, center + 0.07, 35.0, kOpenGrip, error)) {
    return false;
  }
  if (!move_to(id, "descend", x, y, center + 0.015, 45.0, kOpenGrip, error)) {
    return false;
  }
  if (!move_to(id, "close", x, y, center + 0.015, 45.0, close_grip, error)) {
    return false;
  }
  if (!move_to(id, "lift", x, y, center + 0.12, 20.0, close_grip, error)) {
    return false;
  }
  pick_x_ = x;
  pick_y_ = y;
  holding_grip_ = close_grip;
  holding_ = true;
  return true;
}

bool PickPlaceNode::place(std::uint64_t id, const std::string& target,
                          std::string& error) {
  if (!holding_) {
    error = "들고 있는 물체가 없다";
    return false;
  }
  const double hold_grip = holding_grip_ > 0.0 ? holding_grip_ : kPuckGrip;
  const std::string where = lower_copy(target);
  if (where == "body" || where == "deck" || where == "robot" ||
      where == "내몸" || where == "몸") {
    // 로봇 상판 위. 시뮬레이터의 sim/deck_free가 고정물/기존 payload를 피한
    // 빈 자리를 준다. 없으면 기본 자리(0.05, 0)로 시도한다.
    double body_x = 0.05;
    double body_y = 0.0;
    try {
      const krosc::msgs::String response =
          call_service<krosc::msgs::String, krosc::msgs::String>(
              "sim/deck_free", krosc::msgs::String{"{}"});
      if (const auto x = llm::number_field(response.data, "x")) {
        body_x = *x;
      }
      if (const auto y = llm::number_field(response.data, "y")) {
        body_y = *y;
      }
    } catch (const std::exception&) {
      // 시뮬레이터가 없거나 상판 정보를 못 받으면 기본 자리.
    }
    const auto run_body = [&](double x, double y) {
      char extra[96];
      std::snprintf(extra, sizeof(extra),
                    ",\"target\":\"body\",\"x\":%.3f,\"y\":%.3f", x, y);
      publish_status(id, "body_approach", extra);
      if (!move_to(id, "body_approach", x, y, 0.26, 30.0, hold_grip, error)) {
        return false;
      }
      if (!move_to(id, "body_lower", x, y, 0.17, 30.0, hold_grip, error)) {
        return false;
      }
      if (!move_to(id, "body_open", x, y, 0.17, 30.0, kOpenGrip, error)) {
        return false;
      }
      if (!move_to(id, "body_retreat", x, y, 0.26, 20.0, kOpenGrip, error)) {
        return false;
      }
      return true;
    };
    if (!run_body(body_x, body_y)) {
      if (body_x == 0.05 && body_y == 0.0) {
        return false;
      }
      // 빈 자리가 팔 도달 범위 밖일 수 있다. 기본 자리로 한 번 더.
      error.clear();
      publish_status(id, "body_retry",
                     ",\"target\":\"body\",\"x\":0.05,\"y\":0.0");
      if (!run_body(0.05, 0.0)) {
        return false;
      }
    }
    holding_ = false;
    holding_grip_ = 0.0;
    return true;
  }
  if (!move_to(id, "lower", pick_x_, pick_y_, 0.06, 45.0, hold_grip, error)) {
    return false;
  }
  if (!move_to(id, "open", pick_x_, pick_y_, 0.06, 45.0, kOpenGrip, error)) {
    return false;
  }
  if (!move_to(id, "retreat", pick_x_, pick_y_, 0.26, 20.0, kOpenGrip, error)) {
    return false;
  }
  holding_ = false;
  holding_grip_ = 0.0;
  return true;
}

void PickPlaceNode::publish_status(std::uint64_t id, const std::string& stage,
                                   const std::string& extra_json) {
  std::string json = "{\"request_id\":" + std::to_string(id) + ",\"stage\":" +
                     json_quoted(stage);
  if (!extra_json.empty()) {
    json += extra_json;
  }
  json += "}";
  publish<krosc::msgs::String>("pick/status",
                               krosc::msgs::String{std::move(json)});
}

}  // namespace krosc_ws
