#include "krosc/nodes/moveit/move_group.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <thread>
#include <utility>

#include "krosc/tf/math.hpp"
#include "krosc/qos.hpp"
#include "krosc/control/controller_manager.hpp"
#include "krosc/urdf.hpp"

namespace krosc::moveit {

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string trim(const std::string& text) {
  const std::size_t begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const std::size_t end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::optional<std::array<double, 4>> preset_goal(const std::string& text) {
  std::string mode = trim(text);
  for (char& character : mode) {
    character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  }
  if (mode == "home") {
    // all-zero는 바닥을 향하고, STOW와 구분되는 안전한 기본 자세.
    return std::array<double, 4>{0.5000, -1.8000, 1.4000, 0.8000};
  }
  if (mode == "stow") {
    return std::array<double, 4>{0.0, -1.9024, 1.3440, 0.9023};
  }
  return std::nullopt;
}

bool parse_double(const std::string& text, double& value) {
  try {
    std::size_t used = 0;
    const double parsed = std::stod(trim(text), &used);
    if (!std::isfinite(parsed)) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

msgs::PoseStamped make_pose(const std::string& frame, const msgs::Transform& transform, double stamp) {
  msgs::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame;
  pose.pose.position = {transform.translation.x, transform.translation.y, transform.translation.z};
  pose.pose.orientation = transform.rotation;
  return pose;
}

msgs::Path make_path(const std::string& frame,
                     const std::vector<std::pair<std::string, msgs::Transform>>& transforms,
                     double stamp) {
  msgs::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = frame;
  path.poses.reserve(transforms.size());
  for (const auto& item : transforms) {
    path.poses.push_back(make_pose(frame, item.second, stamp));
  }
  return path;
}

}  // namespace

MoveGroup::MoveGroup(Context& context, std::string name, std::map<std::string, ParamValue> parameters,
                     KinematicsRegistry kinematics, PlannerRegistry planners,
                     ControllerManagerRegistry controllers)
    : Node(context, std::move(name), std::move(parameters)),
      base_link_(declare_parameter<std::string>("base_link", "base_link")),
      tip_link_(declare_parameter<std::string>("tip_link", "end_effector_link")),
      joint_names_(declare_parameter<std::vector<std::string>>(
          "joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4"})),
      lower_limits_(declare_parameter<std::vector<double>>(
          "lower_limits", std::vector<double>{-3.141592653589793, -2.20, -2.00, -2.00})),
      upper_limits_(declare_parameter<std::vector<double>>(
          "upper_limits", std::vector<double>{3.141592653589793, 2.20, 2.00, 2.20})),
      max_joint_speed_(declare_parameter<double>("max_joint_speed", 0.35,
                                                 [](double value) { return value > 0.0; })),
      max_joint_accel_(declare_parameter<double>("max_joint_accel", 1.0,
                                                 [](double value) { return value > 0.0; })),
      cartesian_speed_(declare_parameter<double>("cartesian_speed", 0.08,
                                                 [](double value) { return value > 0.0; })),
      cartesian_accel_(declare_parameter<double>("cartesian_accel", 0.25,
                                                 [](double value) { return value > 0.0; })),
      sample_time_(declare_parameter<double>("sample_time", 0.05,
                                             [](double value) { return value > 0.0; })),
      min_tcp_z_(declare_parameter<double>("min_tcp_z", 0.02)),
      arm_stop_topic_(declare_parameter<std::string>("arm_stop_topic", "arm_stop")),
      kinematics_plugin_(declare_parameter<std::string>("kinematics.plugin",
                                                        "krosc_moveit/ArmChainKinematics")),
      planner_plugin_(declare_parameter<std::string>("planner.plugin",
                                                     "krosc_moveit/CartesianPlanner")),
      controller_manager_plugin_(declare_parameter<std::string>(
          "controller_manager.plugin", "krosc_moveit/SimpleControllerManager")),
      switch_service_(declare_parameter<std::string>(
          "controller_manager.switch_service", "controller_manager/switch_controller")),
      trajectory_controller_(declare_parameter<std::string>(
          "controller_manager.trajectory_controller", "joint_trajectory_controller")),
      manual_controller_(declare_parameter<std::string>(
          "controller_manager.manual_controller", "arm_command_controller")),
      kinematics_registry_(std::move(kinematics)),
      planner_registry_(std::move(planners)),
      controller_registry_(std::move(controllers)) {
  if (joint_names_.value().size() != 4 || lower_limits_.value().size() != 4 ||
      upper_limits_.value().size() != 4) {
    logger().error("MoveGroup: joint_names/lower_limits/upper_limits는 4개여야 한다");
  }
  robot_description_subscription_ = subscribe<msgs::String>(
      "robot_description", [this](const msgs::String& message) { on_robot_description(message); },
      QOS_LATCHED);
  joint_states_subscription_ = subscribe<msgs::JointState>(
      "joint_states", [this](const msgs::JointState& message) { on_joint_states(message); },
      QOS_DEFAULT);
  arm_status_subscription_ = subscribe<msgs::String>(
      "arm_status", [this](const msgs::String& message) { on_arm_status(message); }, QOS_DEFAULT);
  create_service<msgs::String, msgs::Bool>(
      Node::name() + "/scrub",
      [this](const msgs::String& message) { return msgs::Bool{on_scrub(message)}; });
  action_ = std::make_unique<ActionServer<msgs::moveit::MoveGroupGoal,
                                          msgs::moveit::MoveGroupFeedback,
                                          msgs::moveit::MoveGroupResult>>(
      context, msgs::moveit::MOVE_ACTION,
      [this](MoveHandle& handle) { return execute_move(handle.goal(), &handle); },
      [this](const msgs::moveit::MoveGroupGoal&) { return !busy_.load(); });
  tick_timer_ = create_timer(0.05, [this] { tick(); });
  status_timer_ = create_timer(1.0, [this] { heartbeat(); });
  logger().info("MoveGroup: base=%s tip=%s joints=%zu kinematics=%s planner=%s controller=%s",
                base_link_.value().c_str(), tip_link_.value().c_str(),
                joint_names_.value().size(), kinematics_plugin_.value().c_str(),
                planner_plugin_.value().c_str(), controller_manager_plugin_.value().c_str());
}

MoveGroup::~MoveGroup() {
  cancel_requested_.store(true);
  if (action_) {
    action_->shutdown();
  }
  std::shared_ptr<FollowJointTrajectoryHandle> follow;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    follow = follow_;
    follow_.reset();
  }
  if (follow) {
    follow->cancel();
  }
}

void MoveGroup::on_robot_description(const msgs::String& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (message.data.empty() || message.data == robot_description_) {
    return;
  }
  robot_description_ = message.data;
  std::string error;
  if (!rebuild_chain(error)) {
    logger().error("MoveGroup: %s", error.c_str());
    status_text_ = "MoveGroup: " + error;
  }
  publish_state_locked();
}

bool MoveGroup::rebuild_chain(std::string& error) {
  chain_ready_ = false;
  try {
    const UrdfRobot robot = parse_urdf(robot_description_);
    auto chain = manipulation::ArmChain::from_urdf(robot, base_link_.value(), tip_link_.value(),
                                                   joint_names_.value(), error);
    if (!chain) {
      return false;
    }
    if (lower_limits_.value().size() == 4) {
      for (std::size_t i = 0; i < 4; ++i) {
        chain->lower[i] = lower_limits_.value()[i];
      }
    }
    if (upper_limits_.value().size() == 4) {
      for (std::size_t i = 0; i < 4; ++i) {
        chain->upper[i] = upper_limits_.value()[i];
      }
    }
    chain_ = std::move(*chain);
    kinematics_ = kinematics_registry_.create(kinematics_plugin_.value());
    planner_ = planner_registry_.create(planner_plugin_.value());
    controller_manager_ = controller_registry_.create(controller_manager_plugin_.value());
    if (!kinematics_ || !planner_ || !controller_manager_) {
      error = "MoveIt 플러그인을 찾을 수 없다: kinematics=" + kinematics_plugin_.value() +
              " planner=" + planner_plugin_.value() + " controller=" +
              controller_manager_plugin_.value();
      return false;
    }
    manipulation::PlanOptions options;
    options.max_joint_speed = max_joint_speed_.value();
    options.max_joint_accel = max_joint_accel_.value();
    options.sample_time = sample_time_.value();
    options.cartesian_speed = cartesian_speed_.value();
    options.cartesian_accel = cartesian_accel_.value();
    kinematics_->initialize(*this, "kinematics", chain_);
    planner_->initialize(*this, "planner", chain_, options);
    controller_manager_->initialize(*this, "controller_manager", joint_names_.value());
    chain_ready_ = true;
    logger().info("MoveGroup: URDF 체인 준비 (segments=%zu)", chain_.segments.size());
    return true;
  } catch (const std::exception& exception) {
    error = std::string("URDF: ") + exception.what();
    return false;
  }
}

void MoveGroup::on_joint_states(const msgs::JointState& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, double> values;
  for (std::size_t i = 0; i < message.name.size() && i < message.position.size(); ++i) {
    values[message.name[i]] = message.position[i];
  }
  std::array<double, 4> joints{};
  for (std::size_t i = 0; i < 4; ++i) {
    const auto found = values.find(joint_names_.value()[i]);
    if (found == values.end()) {
      current_.reset();
      return;
    }
    joints[i] = found->second;
  }
  current_ = joints;
  const auto gripper = values.find("gripper_left_joint");
  if (gripper != values.end()) {
    gripper_ = gripper->second;
  }
}

void MoveGroup::on_arm_status(const msgs::String& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string& text = message.data;
  const std::size_t start = text.find("err=");
  if (start == std::string::npos) {
    return;
  }
  const std::size_t end = text.find(';', start);
  const std::string list =
      text.substr(start + 4, end == std::string::npos ? std::string::npos : end - start - 4);
  std::stringstream stream(list);
  std::string token;
  std::array<int, 5> errors{{-1, -1, -1, -1, -1}};
  std::size_t index = 0;
  while (std::getline(stream, token, ',') && index < errors.size()) {
    token = trim(token);
    if (token == "x" || token.empty()) {
      errors[index] = -1;
    } else {
      try {
        errors[index] = std::stoi(token);
      } catch (...) {
        errors[index] = -1;
      }
    }
    ++index;
  }
  arm_errors_ = errors;
  if (executing_ && has_arm_error_locked()) {
    stop_execution_locked("ARM ERR — Execute 중지", true);
    if (follow_) {
      (void)controller_manager_->cancel(follow_);
    }
  }
}

bool MoveGroup::parse_target(const std::string& text, Target& target, std::string& error) const {
  std::vector<std::string> fields;
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    fields.push_back(trim(token));
  }
  if (fields.size() < 5) {
    error = "moveit_target 형식: x,y,z,pitch_deg,grip_mm[,cartesian]";
    return false;
  }
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double pitch_deg = 0.0;
  double grip_mm = 0.0;
  if (!parse_double(fields[0], x) || !parse_double(fields[1], y) || !parse_double(fields[2], z) ||
      !parse_double(fields[3], pitch_deg) || !parse_double(fields[4], grip_mm)) {
    error = "moveit_target 숫자를 읽을 수 없다";
    return false;
  }
  target.x = x;
  target.y = y;
  target.z = z;
  target.pitch = pitch_deg * kPi / 180.0;
  target.grip = std::clamp(grip_mm / 1000.0, 0.0, 0.03);
  target.cartesian = true;
  if (fields.size() >= 6) {
    target.cartesian = fields[5] != "0" && fields[5] != "false";
  }
  target.preview = false;
  if (fields.size() >= 7) {
    target.preview = fields[6] != "0" && fields[6] != "false";
  }
  return true;
}

bool MoveGroup::current_joints(std::array<double, 4>& joints) const {
  if (!current_) {
    return false;
  }
  joints = *current_;
  return true;
}

bool MoveGroup::has_arm_error_locked() const {
  for (int error : arm_errors_) {
    if (error > 0) {
      return true;
    }
  }
  return false;
}

std::array<int, 5> MoveGroup::arm_errors_locked() const { return arm_errors_; }

bool MoveGroup::solve_ik_locked(const Target& target, const std::array<double, 4>& seed,
                                std::array<double, 4>& goal, std::string& error,
                                bool fast) const {
  if (!kinematics_) {
    error = "kinematics 플러그인이 없다";
    return false;
  }
  KinematicsQuery query;
  query.target = {target.x, target.y, target.z};
  query.pitch_down = target.pitch;
  query.seed = seed;
  query.fast = fast;
  const auto result = kinematics_->search_position_ik(query);
  if (!result.success) {
    error = result.error;
    return false;
  }
  goal = result.solution;
  return true;
}

std::optional<manipulation::Trajectory> MoveGroup::plan_trajectory_locked(
    const std::array<double, 4>& start, const std::array<double, 4>& goal, bool cartesian,
    std::string& error) const {
  if (!planner_) {
    error = "planner 플러그인이 없다";
    return std::nullopt;
  }
  PlanningRequest request;
  request.start = start;
  request.goal = goal;
  request.cartesian = cartesian;
  request.max_joint_speed = max_joint_speed_.value();
  request.max_joint_accel = max_joint_accel_.value();
  request.sample_time = sample_time_.value();
  request.cartesian_speed = cartesian_speed_.value();
  request.cartesian_accel = cartesian_accel_.value();
  auto result = planner_->plan(request);
  if (!result.success) {
    error = result.error;
    return std::nullopt;
  }
  return std::move(result.trajectory);
}

void MoveGroup::publish_status_locked(const std::string& text) {
  status_text_ = text;
  publish("moveit_status", msgs::String{text}, QOS_DEFAULT);
}

void MoveGroup::publish_state_locked() {
  std::ostringstream stream;
  stream.precision(4);
  stream << std::fixed;
  if (chain_ready_ && current_) {
    const msgs::Vector3 tcp = chain_.tcp(*current_);
    stream << "tcp=" << tcp.x << "," << tcp.y << "," << tcp.z;
    stream << ";pitch=" << chain_.pitch_down(*current_) * 180.0 / kPi;
  } else {
    stream << "tcp=nan,nan,nan;pitch=nan";
  }
  stream << ";grip=" << gripper_ * 1000.0;
  const std::size_t points = trajectory_ ? trajectory_->points.size() : 0;
  const double duration = trajectory_ ? trajectory_->duration : 0.0;
  stream << ";points=" << points << ";duration=" << duration << ";index=" << preview_index_;
  stream << ";executing=" << (executing_ ? 1 : 0) << ";has_plan=" << (trajectory_.has_value() ? 1 : 0);
  stream << ";err=" << (has_arm_error_locked() ? 1 : 0);
  publish("moveit_state", msgs::String{stream.str()}, QOS_DEFAULT);
  last_state_ = now();
}

void MoveGroup::publish_target_pose_locked() {
  if (!chain_ready_ || !target_) {
    return;
  }
  msgs::Transform transform{};
  if (goal_) {
    transform = chain_.fk(*goal_);
  } else {
    const msgs::Vector3 point{target_->x, target_->y, target_->z};
    const msgs::Vector3 proximal = tfmath::apply(tfmath::inverse(chain_.base_to_proximal), point);
    const double yaw =
        std::atan2(proximal.y, proximal.x - chain_.proximal_joint_origin.translation.x);
    transform = {{target_->x, target_->y, target_->z}, tfmath::from_rpy(0.0, -target_->pitch, yaw)};
  }
  publish("moveit_target_pose", make_pose(base_link_.value(), transform, now()), QOS_DEFAULT);
}

void MoveGroup::publish_path_locked(
    const std::string& topic,
    const std::vector<std::pair<std::string, msgs::Transform>>& links) {
  publish(topic, make_path(base_link_.value(), links, now()), QOS_DEFAULT);
}

void MoveGroup::publish_preview_locked(int index, bool include_planned_path) {
  if (!chain_ready_) {
    return;
  }
  std::array<double, 4> joints{};
  bool have = false;
  if (trajectory_ && !trajectory_->points.empty()) {
    index = std::clamp(index, 0, static_cast<int>(trajectory_->points.size()) - 1);
    joints = trajectory_->points[static_cast<std::size_t>(index)].position;
    have = true;
  } else if (goal_) {
    joints = *goal_;
    have = true;
  } else if (current_) {
    joints = *current_;
    have = true;
  }
  if (have) {
    publish_path_locked("moveit_arm", chain_.link_transforms(joints));
  }
  if (trajectory_ && !trajectory_->tcp_path.empty()) {
    // 계획 경로는 바뀔 때만 보낸다. 드래그 미리보기마다 100+ 점을 다시 보내면
    // 웹 소켓이 막혀 TCP 마커가 버벅인다.
    if (include_planned_path) {
      std::vector<std::pair<std::string, msgs::Transform>> points;
      points.reserve(trajectory_->tcp_path.size());
      for (const auto& point : trajectory_->tcp_path) {
        points.emplace_back("tcp", msgs::Transform{point, {}});
      }
      publish_path_locked("moveit_path", points);
    }
  } else if (!preview_path_.empty()) {
    std::vector<std::pair<std::string, msgs::Transform>> points;
    points.reserve(preview_path_.size());
    for (const auto& point : preview_path_) {
      points.emplace_back("tcp", msgs::Transform{point, {}});
    }
    publish_path_locked("moveit_path", points);
  }
  publish_target_pose_locked();
}

bool MoveGroup::plan_locked(std::string& error) {
  if (!chain_ready_) {
    publish_status_locked("Plan 실패: robot_description/URDF 체인 대기 중");
    publish_state_locked();
    return false;
  }
  if (!current_) {
    publish_status_locked("Plan 실패: joint_states 대기 중");
    publish_state_locked();
    return false;
  }
  if (!target_ && !joint_goal_) {
    publish_status_locked("Plan 실패: moveit_target/moveit_preset이 없다");
    publish_state_locked();
    return false;
  }
  if (has_arm_error_locked()) {
    trajectory_.reset();
    goal_.reset();
    publish_status_locked("Plan 실패: ARM ERR — 먼저 Reset (AR)");
    publish_target_pose_locked();
    publish_state_locked();
    return false;
  }
  std::array<double, 4> goal{};
  const bool joint_mode = joint_goal_.has_value();
  if (joint_mode) {
    goal = *joint_goal_;
  } else {
    std::string ik_error;
    bool ik_ok = false;
    if (goal_) {
      ik_ok = solve_ik_locked(*target_, *goal_, goal, ik_error, true);
    }
    if (!ik_ok) {
      ik_ok = solve_ik_locked(*target_, *current_, goal, ik_error, true);
    }
    if (!ik_ok) {
      ik_ok = solve_ik_locked(*target_, *current_, goal, ik_error, false);
    }
    if (!ik_ok) {
      trajectory_.reset();
      goal_.reset();
      preview_path_.clear();
      error = ik_error;
      publish_status_locked("Plan 실패: " + error);
      publish_target_pose_locked();
      publish_state_locked();
      return false;
    }
  }
  goal_ = goal;
  if (!joint_mode && target_->preview) {
    trajectory_.reset();
    preview_path_.clear();
    const msgs::Vector3 start_tcp = chain_.tcp(*current_);
    const msgs::Vector3 goal_tcp = chain_.tcp(goal);
    constexpr int kPreviewPoints = 16;
    for (int i = 0; i <= kPreviewPoints; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(kPreviewPoints);
      preview_path_.push_back({start_tcp.x + (goal_tcp.x - start_tcp.x) * t,
                               start_tcp.y + (goal_tcp.y - start_tcp.y) * t,
                               start_tcp.z + (goal_tcp.z - start_tcp.z) * t});
    }
    preview_index_ = 0;
    publish_preview_locked(0, false);
    publish_state_locked();
    publish_status_locked("Preview: IK ok — Plan을 누르면 궤적을 계산한다");
    return true;
  }
  preview_path_.clear();
  auto trajectory =
      plan_trajectory_locked(*current_, goal, joint_mode ? false : target_->cartesian, error);
  if (!trajectory) {
    trajectory_.reset();
    goal_.reset();
    publish_status_locked("Plan 실패: " + error);
    publish_target_pose_locked();
    publish_state_locked();
    return false;
  }
  trajectory_ = std::move(*trajectory);
  preview_index_ = static_cast<int>(trajectory_->points.size()) - 1;
  publish_preview_locked(preview_index_);
  publish_state_locked();
  std::ostringstream status;
  status << "Plan OK: " << trajectory_->points.size() << " pts, " << std::fixed;
  status.precision(2);
  status << trajectory_->duration << " s, " << (trajectory_->cartesian ? "Cartesian" : "joint-space");
  if (trajectory_->fallback) {
    status << "\nCartesian 실패 → joint-space 대체";
  }
  status << " [" << planner_->type() << "]";
  publish_status_locked(status.str());
  return true;
}

bool MoveGroup::goal_from_request_locked(const msgs::moveit::MoveGroupGoal& request,
                                         std::array<double, 4>& goal, std::string& error) const {
  if (!current_) {
    error = "joint_states가 없다";
    return false;
  }
  if (const auto preset = preset_goal(request.preset)) {
    goal = *preset;
    return true;
  }
  if (request.joint_goal.size() == 4) {
    for (std::size_t i = 0; i < 4; ++i) {
      if (request.joint_goal[i] < chain_.lower[i] - 1e-6 ||
          request.joint_goal[i] > chain_.upper[i] + 1e-6) {
        error = "joint_goal이 관절 한계를 벗어난다";
        return false;
      }
      goal[i] = request.joint_goal[i];
    }
    return true;
  }
  Target target;
  target.x = request.target_pose.pose.position.x;
  target.y = request.target_pose.pose.position.y;
  target.z = request.target_pose.pose.position.z;
  if (target.z < min_tcp_z_.value()) {
    error = "target z가 바닥/작업영역보다 낮다";
    return false;
  }
  const auto rpy = tfmath::to_rpy(request.target_pose.pose.orientation);
  target.pitch = -rpy.pitch;
  target.grip = request.grip >= 0.0 ? std::clamp(request.grip, 0.0, 0.03) : gripper_;
  target.cartesian = request.cartesian;
  target.preview = false;
  std::string ik_error;
  if (solve_ik_locked(target, *current_, goal, ik_error, true)) {
    return true;
  }
  error = ik_error;
  return false;
}

msgs::control::JointTrajectory MoveGroup::to_message_locked(
    const manipulation::Trajectory& trajectory) const {
  msgs::control::JointTrajectory message;
  message.header = msgs::Header{now(), base_link_.value()};
  message.joint_names = joint_names_.value();
  message.joint_names.push_back("gripper_left_joint");
  const double grip = action_grip_ >= 0.0 ? action_grip_ : (target_ ? target_->grip : gripper_);
  message.points.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points) {
    msgs::control::JointTrajectoryPoint target;
    target.positions.assign(point.position.begin(), point.position.end());
    target.positions.push_back(grip);
    target.time_from_start = point.time;
    message.points.push_back(std::move(target));
  }
  return message;
}

void MoveGroup::on_target(const msgs::String& message) {
  Target target;
  std::string error;
  if (!parse_target(message.data, target, error)) {
    std::lock_guard<std::mutex> lock(mutex_);
    publish_status_locked("Target 오류: " + error);
    return;
  }
  if (target.z < min_tcp_z_.value()) {
    std::lock_guard<std::mutex> lock(mutex_);
    publish_status_locked("Target 거부: 바닥/작업영역보다 낮다 (z=" +
                          std::to_string(target.z) + ")");
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (executing_ || busy_.load()) {
    publish_status_locked("Target 거부: Execute 중이다");
    return;
  }
  action_grip_ = -1.0;
  target_ = target;
  joint_goal_.reset();
  goal_.reset();
  trajectory_.reset();
  preview_index_ = 0;
  if (!plan_locked(error)) {
    return;
  }
}

void MoveGroup::on_preset(const msgs::String& message) {
  const auto goal = preset_goal(message.data);
  if (!goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    publish_status_locked("Preset 오류: home|stow");
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (executing_ || busy_.load()) {
    publish_status_locked("Preset 거부: Execute 중이다");
    return;
  }
  action_grip_ = -1.0;
  target_.reset();
  joint_goal_ = *goal;
  goal_.reset();
  trajectory_.reset();
  preview_index_ = 0;
  std::string error;
  (void)plan_locked(error);
}

void MoveGroup::on_plan(const msgs::Empty&) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (executing_ || busy_.load()) {
    publish_status_locked("Plan 거부: Execute 중이다");
    return;
  }
  if (target_) {
    target_->preview = false;
  }
  std::string error;
  (void)plan_locked(error);
}

void MoveGroup::ensure_trajectory_controller() {
  if (switch_service_.value().empty() || trajectory_controller_.value().empty()) {
    return;
  }
  krosc::control::SwitchControllerRequest request;
  request.start_controllers.push_back(trajectory_controller_.value());
  if (!manual_controller_.value().empty()) {
    request.stop_controllers.push_back(manual_controller_.value());
  }
  request.strictness = 1;
  try {
    const auto response =
        call_service<krosc::control::SwitchControllerRequest, krosc::control::SwitchControllerResponse>(
            switch_service_.value(), request);
    if (!response.ok) {
      logger().warn("MoveGroup: %s 활성화 실패: %s", trajectory_controller_.value().c_str(),
                    response.message.c_str());
    }
  } catch (const std::exception& error) {
    logger().warn("MoveGroup: controller switch 실패: %s", error.what());
  }
}

void MoveGroup::on_execute(const msgs::Empty&) {
  action_grip_ = -1.0;
  // controller_manager 서비스는 MoveGroup mutex 밖에서 호출한다.
  // 반대로 잡으면 controller_manager update(자기 mutex) -> joint_states publish ->
  // MoveGroup mutex 대기와 락 순서가 뒤집혀 데드락이 난다.
  ensure_trajectory_controller();
  std::lock_guard<std::mutex> lock(mutex_);
  if (busy_.exchange(true)) {
    publish_status_locked("Execute: 이미 실행 중");
    return;
  }
  if (!trajectory_ || trajectory_->points.empty()) {
    busy_.store(false);
    publish_status_locked("Execute 실패: 계획이 없다");
    return;
  }
  if (!current_) {
    busy_.store(false);
    publish_status_locked("Execute 실패: joint_states가 없다");
    return;
  }
  if (has_arm_error_locked()) {
    busy_.store(false);
    publish_status_locked("Execute 실패: ARM ERR — 먼저 Reset (AR)");
    return;
  }
  const auto& first = trajectory_->points.front().position;
  double difference = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    difference = std::max(difference, std::abs(first[i] - (*current_)[i]));
  }
  if (difference > 0.08) {
    busy_.store(false);
    publish_status_locked("Execute 거부: 현재 자세가 계획 시작과 다르다 (replan)");
    return;
  }
  if (!controller_manager_) {
    busy_.store(false);
    publish_status_locked("Execute 실패: controller_manager 플러그인이 없다");
    return;
  }
  follow_ = controller_manager_->send(to_message_locked(*trajectory_));
  if (!follow_ || !follow_->accepted()) {
    follow_.reset();
    busy_.store(false);
    publish_status_locked("Execute 실패: controller가 goal을 거부했다");
    return;
  }
  cancel_requested_.store(false);
  executing_ = true;
  action_owned_ = false;
  execution_start_ = now();
  preview_index_ = 0;
  publish_status_locked("Execute 시작");
  publish_state_locked();
}

void MoveGroup::stop_execution_locked(const std::string& reason, bool publish_stop) {
  executing_ = false;
  if (publish_stop) {
    publish(arm_stop_topic_.value(), msgs::Empty{}, QOS_DEFAULT);
  }
  publish_status_locked(reason);
  publish_state_locked();
}

void MoveGroup::on_cancel(const msgs::Empty&) {
  std::lock_guard<std::mutex> lock(mutex_);
  cancel_requested_.store(true);
  if (follow_ && controller_manager_) {
    (void)controller_manager_->cancel(follow_);
  }
  stop_execution_locked("Cancel: 정지, 다시 Plan 필요", true);
  follow_.reset();
  if (!action_owned_) {
    busy_.store(false);
  }
}

bool MoveGroup::on_scrub(const msgs::String& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (executing_ || busy_.load()) {
    publish_status_locked("Preview: Execute 중에는 scrub할 수 없다");
    return false;
  }
  if (!trajectory_ || trajectory_->points.empty()) {
    publish_status_locked("Preview: 계획이 없다");
    return false;
  }
  int index = 0;
  try {
    index = std::stoi(trim(message.data));
  } catch (...) {
    publish_status_locked("Preview: index를 읽을 수 없다");
    return false;
  }
  index = std::clamp(index, 0, static_cast<int>(trajectory_->points.size()) - 1);
  preview_index_ = index;
  publish_preview_locked(index);
  publish_state_locked();
  std::ostringstream status;
  status << "Preview " << (index + 1) << " / " << trajectory_->points.size() << "  t=";
  status.precision(2);
  status << std::fixed << trajectory_->points[static_cast<std::size_t>(index)].time << " s";
  publish_status_locked(status.str());
  return true;
}

void MoveGroup::tick() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!executing_ || !trajectory_ || trajectory_->points.empty()) {
    return;
  }
  const auto& points = trajectory_->points;
  const double duration = trajectory_->duration;
  const double elapsed = std::clamp(now() - execution_start_, 0.0, duration);
  std::size_t index = 0;
  while (index + 1 < points.size() && points[index + 1].time <= elapsed) {
    ++index;
  }
  preview_index_ = static_cast<int>(index);
  if (!action_owned_ && follow_ && follow_->status() != GoalStatus::Accepted) {
    const ActionResult action_result = follow_->result(1.0);
    int code = msgs::control::FollowJointTrajectoryResult::INVALID_GOAL;
    std::string error;
    if (action_result.result.has_value()) {
      try {
        const auto result =
            std::any_cast<msgs::control::FollowJointTrajectoryResult>(action_result.result);
        code = result.error_code;
        error = result.error_string;
      } catch (const std::bad_any_cast&) {
      }
    }
    follow_.reset();
    executing_ = false;
    action_owned_ = false;
    busy_.store(false);
    if (code == msgs::control::FollowJointTrajectoryResult::SUCCESSFUL) {
      publish_status_locked("Execute 완료");
    } else {
      publish_status_locked("Execute 실패: " + error);
    }
    publish_preview_locked(preview_index_);
    publish_state_locked();
    return;
  }
  if (now() - last_preview_ >= 0.1) {
    publish_preview_locked(preview_index_, false);
    last_preview_ = now();
  }
  if (now() - last_state_ >= 0.2) {
    publish_state_locked();
  }
  if (now() - last_status_ >= 0.2) {
    std::ostringstream status;
    status << "Execute: " << std::fixed;
    status.precision(2);
    status << elapsed << " / " << duration << " s ("
           << (duration > 1e-9 ? 100.0 * elapsed / duration : 100.0) << "%)";
    publish_status_locked(status.str());
    last_status_ = now();
  }
}

void MoveGroup::heartbeat() {
  std::lock_guard<std::mutex> lock(mutex_);
  publish_state_locked();
  publish_status_locked(status_text_);
  last_status_ = now();
  last_preview_ = now();
  // 아직 목표가 없으면 현재 TCP로 목표 마커를 초기화한다.
  // (안 하면 moveit_target_pose가 발행되지 않아 Viz에서 TCP 원반이 없어 드래그를 시작할 수 없다)
  if (!target_ && chain_ready_ && current_) {
    Target target;
    const msgs::Vector3 tcp = chain_.tcp(*current_);
    target.x = tcp.x;
    target.y = tcp.y;
    target.z = tcp.z;
    target.pitch = chain_.pitch_down(*current_);
    target.grip = gripper_;
    target.cartesian = true;
    target.preview = false;
    target_ = target;
  }
  if ((trajectory_ && !trajectory_->points.empty()) || goal_ || !preview_path_.empty()) {
    publish_preview_locked(preview_index_, false);
  } else {
    publish_target_pose_locked();
  }
}

msgs::moveit::MoveGroupResult MoveGroup::execute_move(const msgs::moveit::MoveGroupGoal& request,
                                                      MoveHandle* handle) {
  action_grip_ = request.grip >= 0.0 ? std::clamp(request.grip, 0.0, 0.03) : -1.0;
  msgs::moveit::MoveGroupResult result;
  if (busy_.exchange(true)) {
    result.error_code = msgs::moveit::MoveGroupResult::CONTROLLER_FAILED;
    result.error_msg = "이미 실행 중이다";
    return result;
  }
  cancel_requested_.store(false);
  std::array<double, 4> start{};
  std::array<double, 4> goal{};
  std::string error;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!chain_ready_ || !current_) {
      busy_.store(false);
      result.error_code = msgs::moveit::MoveGroupResult::UNKNOWN;
      result.error_msg = !chain_ready_ ? "URDF 체인 대기 중" : "joint_states 대기 중";
      publish_status_locked("Move: " + result.error_msg);
      return result;
    }
    if (has_arm_error_locked()) {
      busy_.store(false);
      result.error_code = msgs::moveit::MoveGroupResult::INVALID_GOAL;
      result.error_msg = "ARM ERR — 먼저 Reset (AR)";
      publish_status_locked("Move: " + result.error_msg);
      return result;
    }
    start = *current_;
    if (!goal_from_request_locked(request, goal, error)) {
      busy_.store(false);
      result.error_code = msgs::moveit::MoveGroupResult::INVALID_GOAL;
      result.error_msg = error;
      publish_status_locked("Move 실패: " + error);
      return result;
    }
    auto trajectory = plan_trajectory_locked(start, goal, request.cartesian, error);
    if (!trajectory) {
      busy_.store(false);
      result.error_code = msgs::moveit::MoveGroupResult::PLANNING_FAILED;
      result.error_msg = error;
      publish_status_locked("Move 실패: " + error);
      return result;
    }
    trajectory_ = std::move(*trajectory);
    goal_ = goal;
    preview_index_ = static_cast<int>(trajectory_->points.size()) - 1;
    publish_preview_locked(preview_index_);
    publish_state_locked();
    result.trajectory = to_message_locked(*trajectory_);
    if (request.plan_only) {
      busy_.store(false);
      publish_status_locked("Move: plan_only");
      return result;
    }
    if (!controller_manager_) {
      busy_.store(false);
      result.error_code = msgs::moveit::MoveGroupResult::CONTROLLER_FAILED;
      result.error_msg = "controller_manager 플러그인이 없다";
      return result;
    }
    follow_ = controller_manager_->send(result.trajectory);
    if (!follow_ || !follow_->accepted()) {
      follow_.reset();
      busy_.store(false);
      result.error_code = msgs::moveit::MoveGroupResult::CONTROLLER_FAILED;
      result.error_msg = "controller가 goal을 거부했다";
      publish_status_locked("Move 실패: " + result.error_msg);
      return result;
    }
    executing_ = true;
    action_owned_ = true;
    execution_start_ = now();
    publish_status_locked("Execute 시작 (move_action)");
  }
  double last_feedback = -1e18;
  double duration = 0.0;
  double started = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    duration = trajectory_ ? trajectory_->duration : 0.0;
    started = execution_start_;
  }
  for (;;) {
    if (handle != nullptr && handle->is_cancel_requested()) {
      cancel_requested_.store(true);
    }
    if (cancel_requested_.load()) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (follow_ && controller_manager_) {
        (void)controller_manager_->cancel(follow_);
      }
      follow_.reset();
      executing_ = false;
      action_owned_ = false;
      busy_.store(false);
      publish_status_locked("Cancel: 정지, 다시 Plan 필요");
      result.error_code = msgs::moveit::MoveGroupResult::CANCELLED;
      result.error_msg = "취소";
      return result;
    }
    std::shared_ptr<FollowJointTrajectoryHandle> follow;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      follow = follow_;
    }
    if (!follow || follow->status() != GoalStatus::Accepted) {
      break;
    }
    const double current = now();
    if (handle != nullptr && current - last_feedback >= 0.1) {
      last_feedback = current;
      msgs::moveit::MoveGroupFeedback feedback;
      feedback.state = "executing";
      feedback.progress = duration > 1e-9 ? std::clamp((current - started) / duration, 0.0, 1.0) : 1.0;
      handle->publish_feedback(feedback);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::shared_ptr<FollowJointTrajectoryHandle> follow;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    follow = follow_;
    follow_.reset();
    executing_ = false;
    action_owned_ = false;
    busy_.store(false);
  }
  int code = msgs::control::FollowJointTrajectoryResult::INVALID_GOAL;
  std::string message;
  if (follow) {
    const ActionResult action_result = follow->result(1.0);
    if (action_result.result.has_value()) {
      try {
        const auto controller_result =
            std::any_cast<msgs::control::FollowJointTrajectoryResult>(action_result.result);
        code = controller_result.error_code;
        message = controller_result.error_string;
      } catch (const std::bad_any_cast&) {
      }
    }
  } else {
    message = "controller goal이 사라졌다";
  }
  std::lock_guard<std::mutex> lock(mutex_);
  publish_state_locked();
  if (code == msgs::control::FollowJointTrajectoryResult::SUCCESSFUL) {
    publish_status_locked("Execute 완료");
    result.error_code = msgs::moveit::MoveGroupResult::NONE;
  } else {
    publish_status_locked("Execute 실패: " + message);
    result.error_code = msgs::moveit::MoveGroupResult::CONTROLLER_FAILED;
    result.error_msg = message;
  }
  return result;
}

}  // namespace krosc::moveit
