#include "krosc/nodes/moveit_lite.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <utility>

#include "krosc/math.hpp"
#include "krosc/qos.hpp"
#include "krosc/urdf.hpp"

namespace krosc::nodes {

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
                     const std::vector<std::pair<std::string, msgs::Transform>>& transforms, double stamp) {
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

MoveItLite::MoveItLite(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
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
      arm_command_topic_(declare_parameter<std::string>("arm_command_topic", "arm_joint_command")),
      arm_stop_topic_(declare_parameter<std::string>("arm_stop_topic", "arm_stop")) {
  if (joint_names_.value().size() != 4 || lower_limits_.value().size() != 4 ||
      upper_limits_.value().size() != 4) {
    logger().error("MoveItLite: joint_names/lower_limits/upper_limits는 4개여야 한다");
  }
  robot_description_subscription_ = subscribe<msgs::String>(
      "robot_description", [this](const msgs::String& message) { on_robot_description(message); },
      QOS_LATCHED);
  joint_states_subscription_ = subscribe<msgs::JointState>(
      "joint_states", [this](const msgs::JointState& message) { on_joint_states(message); },
      QOS_DEFAULT);
  arm_status_subscription_ = subscribe<msgs::String>(
      "arm_status", [this](const msgs::String& message) { on_arm_status(message); }, QOS_DEFAULT);
  target_subscription_ = subscribe<msgs::String>(
      "moveit_target", [this](const msgs::String& message) { on_target(message); }, QOS_DEFAULT);
  plan_subscription_ = subscribe<msgs::Empty>(
      "moveit_plan", [this](const msgs::Empty& message) { on_plan(message); }, QOS_DEFAULT);
  execute_subscription_ = subscribe<msgs::Empty>(
      "moveit_execute", [this](const msgs::Empty& message) { on_execute(message); }, QOS_DEFAULT);
  cancel_subscription_ = subscribe<msgs::Empty>(
      "moveit_cancel", [this](const msgs::Empty& message) { on_cancel(message); }, QOS_DEFAULT);
  scrub_subscription_ = subscribe<msgs::String>(
      "moveit_scrub", [this](const msgs::String& message) { on_scrub(message); }, QOS_DEFAULT);
  tick_timer_ = create_timer(0.05, [this] { tick(); });
  status_timer_ = create_timer(1.0, [this] { heartbeat(); });
  logger().info("MoveItLite: base=%s tip=%s joints=%zu speed=%.2f/%.2f cart=%.3f",
                base_link_.value().c_str(), tip_link_.value().c_str(), joint_names_.value().size(),
                max_joint_speed_.value(), max_joint_accel_.value(), cartesian_speed_.value());
}

void MoveItLite::on_robot_description(const msgs::String& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (message.data.empty() || message.data == robot_description_) {
    return;
  }
  robot_description_ = message.data;
  std::string error;
  if (!rebuild_chain(error)) {
    logger().error("MoveItLite: %s", error.c_str());
    status_text_ = "MoveIt: " + error;
  }
}

bool MoveItLite::rebuild_chain(std::string& error) {
  chain_ready_ = false;
  try {
    const UrdfRobot robot = parse_urdf(robot_description_);
    auto chain = manipulation::ArmChain::from_urdf(robot, base_link_.value(), tip_link_.value(),
                                                   joint_names_.value(), error);
    if (!chain) {
      return false;
    }
    chain->lower = {-3.141592653589793, -2.20, -2.00, -2.00};
    chain->upper = {3.141592653589793, 2.20, 2.00, 2.20};
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
    chain_ready_ = true;
    logger().info("MoveItLite: URDF 체인 준비 (segments=%zu)", chain_.segments.size());
    return true;
  } catch (const std::exception& exception) {
    error = std::string("URDF: ") + exception.what();
    return false;
  }
}

void MoveItLite::on_joint_states(const msgs::JointState& message) {
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

void MoveItLite::on_arm_status(const msgs::String& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string& text = message.data;
  const std::size_t start = text.find("err=");
  if (start == std::string::npos) {
    return;
  }
  const std::size_t end = text.find(';', start);
  const std::string list = text.substr(start + 4, end == std::string::npos ? std::string::npos : end - start - 4);
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
  }
}

bool MoveItLite::parse_target(const std::string& text, Target& target, std::string& error) const {
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
  return true;
}

bool MoveItLite::current_joints(std::array<double, 4>& joints) const {
  if (!current_) {
    return false;
  }
  joints = *current_;
  return true;
}

bool MoveItLite::has_arm_error_locked() const {
  for (int error : arm_errors_) {
    if (error > 0) {
      return true;
    }
  }
  return false;
}

std::array<int, 5> MoveItLite::arm_errors_locked() const { return arm_errors_; }

void MoveItLite::publish_status_locked(const std::string& text) {
  status_text_ = text;
  publish("moveit_status", msgs::String{text}, QOS_DEFAULT);
}

void MoveItLite::publish_state_locked() {
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

void MoveItLite::publish_target_pose_locked() {
  if (!chain_ready_ || !target_) {
    return;
  }
  msgs::Transform transform{};
  if (goal_) {
    transform = chain_.fk(*goal_);
  } else {
    const msgs::Vector3 point{target_->x, target_->y, target_->z};
    const msgs::Vector3 proximal = tfmath::apply(tfmath::inverse(chain_.base_to_proximal), point);
    const double yaw = std::atan2(proximal.y, proximal.x - chain_.proximal_joint_origin.translation.x);
    transform = {{target_->x, target_->y, target_->z}, tfmath::from_rpy(0.0, target_->pitch, yaw)};
  }
  publish("moveit_target_pose", make_pose(base_link_.value(), transform, now()), QOS_DEFAULT);
}

void MoveItLite::publish_path_locked(
    const std::string& topic,
    const std::vector<std::pair<std::string, msgs::Transform>>& links) {
  publish(topic, make_path(base_link_.value(), links, now()), QOS_DEFAULT);
}

void MoveItLite::publish_preview_locked(int index) {
  if (!chain_ready_) {
    return;
  }
  std::array<double, 4> joints{};
  bool have = false;
  if (trajectory_ && !trajectory_->points.empty()) {
    index = std::clamp(index, 0, static_cast<int>(trajectory_->points.size()) - 1);
    joints = trajectory_->points[static_cast<std::size_t>(index)].position;
    have = true;
  } else if (current_) {
    joints = *current_;
    have = true;
  }
  if (have) {
    publish_path_locked("moveit_arm", chain_.link_transforms(joints));
  }
  if (trajectory_ && !trajectory_->tcp_path.empty()) {
    std::vector<std::pair<std::string, msgs::Transform>> points;
    points.reserve(trajectory_->tcp_path.size());
    for (const auto& point : trajectory_->tcp_path) {
      points.emplace_back("tcp", msgs::Transform{point, {}});
    }
    publish_path_locked("moveit_path", points);
  }
  publish_target_pose_locked();
}

bool MoveItLite::plan_locked(std::string& error) {
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
  if (!target_) {
    publish_status_locked("Plan 실패: moveit_target이 없다");
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
  if (!chain_.ik({target_->x, target_->y, target_->z}, target_->pitch, *current_, goal, error)) {
    trajectory_.reset();
    goal_.reset();
    publish_status_locked("Plan 실패: " + error);
    publish_target_pose_locked();
    publish_state_locked();
    return false;
  }
  manipulation::PlanOptions options;
  options.max_joint_speed = max_joint_speed_.value();
  options.max_joint_accel = max_joint_accel_.value();
  options.sample_time = sample_time_.value();
  options.cartesian = target_->cartesian;
  options.cartesian_speed = cartesian_speed_.value();
  options.cartesian_accel = cartesian_accel_.value();
  auto trajectory = manipulation::plan_trajectory(chain_, *current_, goal, options, error);
  if (!trajectory) {
    trajectory_.reset();
    goal_.reset();
    publish_status_locked("Plan 실패: " + error);
    publish_target_pose_locked();
    publish_state_locked();
    return false;
  }
  trajectory_ = std::move(*trajectory);
  goal_ = goal;
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
  publish_status_locked(status.str());
  return true;
}

void MoveItLite::on_target(const msgs::String& message) {
  Target target;
  std::string error;
  if (!parse_target(message.data, target, error)) {
    std::lock_guard<std::mutex> lock(mutex_);
    publish_status_locked("Target 오류: " + error);
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (executing_) {
    publish_status_locked("Target 거부: Execute 중이다");
    return;
  }
  target_ = target;
  goal_.reset();
  trajectory_.reset();
  preview_index_ = 0;
  if (!plan_locked(error)) {
    return;
  }
}

void MoveItLite::on_plan(const msgs::Empty&) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (executing_) {
    publish_status_locked("Plan 거부: Execute 중이다");
    return;
  }
  std::string error;
  if (!plan_locked(error)) {
    return;
  }
}

void MoveItLite::on_execute(const msgs::Empty&) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (executing_) {
    publish_status_locked("Execute: 이미 실행 중");
    return;
  }
  if (!trajectory_ || trajectory_->points.empty()) {
    publish_status_locked("Execute 실패: 계획이 없다");
    return;
  }
  if (!current_) {
    publish_status_locked("Execute 실패: joint_states가 없다");
    return;
  }
  if (has_arm_error_locked()) {
    publish_status_locked("Execute 실패: ARM ERR — 먼저 Reset (AR)");
    return;
  }
  const auto& first = trajectory_->points.front().position;
  double difference = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    difference = std::max(difference, std::abs(first[i] - (*current_)[i]));
  }
  if (difference > 0.08) {
    publish_status_locked("Execute 거부: 현재 자세가 계획 시작과 다르다 (replan)");
    return;
  }
  executing_ = true;
  execution_start_ = now();
  preview_index_ = 0;
  publish_status_locked("Execute 시작");
  publish_state_locked();
}

void MoveItLite::stop_execution_locked(const std::string& reason, bool publish_stop) {
  executing_ = false;
  if (publish_stop) {
    publish(arm_stop_topic_.value(), msgs::Empty{}, QOS_DEFAULT);
  }
  publish_status_locked(reason);
  publish_state_locked();
}

void MoveItLite::on_cancel(const msgs::Empty&) {
  std::lock_guard<std::mutex> lock(mutex_);
  stop_execution_locked("Cancel: 정지, 다시 Plan 필요", true);
  trajectory_.reset();
  goal_.reset();
  preview_index_ = 0;
  publish_target_pose_locked();
  publish_state_locked();
}

void MoveItLite::on_scrub(const msgs::String& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (executing_) {
    publish_status_locked("Preview: Execute 중에는 scrub할 수 없다");
    return;
  }
  if (!trajectory_ || trajectory_->points.empty()) {
    publish_status_locked("Preview: 계획이 없다");
    return;
  }
  int index = 0;
  try {
    index = std::stoi(trim(message.data));
  } catch (...) {
    publish_status_locked("Preview: index를 읽을 수 없다");
    return;
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
}

void MoveItLite::tick() {
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
  std::array<double, 4> position = points[index].position;
  if (index + 1 < points.size()) {
    const double segment = points[index + 1].time - points[index].time;
    if (segment > 1e-9) {
      const double ratio = std::clamp((elapsed - points[index].time) / segment, 0.0, 1.0);
      for (std::size_t i = 0; i < 4; ++i) {
        position[i] = points[index].position[i] +
                      (points[index + 1].position[i] - points[index].position[i]) * ratio;
      }
    }
  }
  msgs::JointState command;
  for (std::size_t i = 0; i < 4; ++i) {
    command.name.push_back(joint_names_.value()[i]);
    command.position.push_back(position[i]);
  }
  command.name.push_back("gripper_left_joint");
  command.position.push_back(target_ ? target_->grip : gripper_);
  publish(arm_command_topic_.value(), command, QOS_DEFAULT);
  preview_index_ = static_cast<int>(index);
  if (now() - last_preview_ >= 0.1) {
    publish_preview_locked(preview_index_);
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
  if (elapsed >= duration - 1e-9) {
    stop_execution_locked("Execute 완료", false);
    publish_preview_locked(preview_index_);
  }
}

void MoveItLite::heartbeat() {
  std::lock_guard<std::mutex> lock(mutex_);
  publish_state_locked();
  publish_status_locked(status_text_);
  last_status_ = now();
  last_preview_ = now();
  if (trajectory_ && !trajectory_->points.empty()) {
    publish_preview_locked(preview_index_);
  } else {
    publish_target_pose_locked();
  }
}

}  // namespace krosc::nodes
