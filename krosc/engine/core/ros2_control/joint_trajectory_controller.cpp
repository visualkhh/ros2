#include "krosc/ros2_control/joint_trajectory_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include "krosc/math.hpp"
#include "krosc/qos.hpp"

namespace krosc::ros2_control {

JointTrajectoryController::~JointTrajectoryController() {
  if (action_) {
    action_->shutdown();
  }
}

bool JointTrajectoryController::on_init(ControllerContext& context) {
  if (!ControllerInterface::on_init(context)) {
    return false;
  }
  joints_ = context.node->declare_parameter<std::vector<std::string>>(
      context.name + ".joints", std::vector<std::string>{});
  if (joints_.empty()) {
    context.node->logger().error("joint_trajectory_controller: %s.joints가 비어 있다",
                                 context.name.c_str());
    return false;
  }
  for (const auto& joint : joints_) {
    auto command = context.command(joint + "/position");
    auto state = context.state(joint + "/position");
    if (!command || !state) {
      context.node->logger().error("joint_trajectory_controller: %s/position 인터페이스가 없다",
                                   joint.c_str());
      return false;
    }
    commands_.push_back(std::move(command));
    states_.push_back(std::move(state));
  }
  action_name_ = context.node->declare_parameter<std::string>(context.name + ".action_name",
                                                              "follow_joint_trajectory");
  state_topic_ = context.node->declare_parameter<std::string>(context.name + ".state_topic",
                                                              context.name + "/state");
  goal_time_tolerance_ = context.node->declare_parameter<double>(
      context.name + ".goal_time_tolerance", 0.5, [](double value) { return value >= 0.0; });
  std::string action_name = action_name_.value();
  if (action_name.find('/') == std::string::npos) {
    action_name = context.name + "/" + action_name;
  }
  action_ = std::make_unique<ActionServer<msgs::control::FollowJointTrajectoryGoal,
                                          msgs::control::FollowJointTrajectoryFeedback,
                                          msgs::control::FollowJointTrajectoryResult>>(
      context.node->context(), action_name,
      [this](Handle& handle) { return execute(handle); },
      [this](const msgs::control::FollowJointTrajectoryGoal&) { return active_.load(); });
  actual_.assign(joints_.size(), 0.0);
  desired_.assign(joints_.size(), 0.0);
  return true;
}

bool JointTrajectoryController::on_configure() {
  actual_.assign(joints_.size(), 0.0);
  return true;
}

bool JointTrajectoryController::on_activate() {
  active_.store(true);
  apply_hold();
  return true;
}

bool JointTrajectoryController::on_deactivate() {
  active_.store(false);
  running_.store(false);
  done_.store(true);
  apply_hold();
  return true;
}

bool JointTrajectoryController::on_cleanup() {
  if (action_) {
    action_->shutdown();
    action_.reset();
  }
  return true;
}

std::vector<std::string> JointTrajectoryController::command_interface_names() const {
  std::vector<std::string> names;
  names.reserve(joints_.size());
  for (const auto& joint : joints_) {
    names.push_back(joint + "/position");
  }
  return names;
}

void JointTrajectoryController::apply_hold() {
  std::lock_guard<std::mutex> lock(mutex_);
  hold_.resize(commands_.size());
  for (std::size_t i = 0; i < commands_.size(); ++i) {
    hold_[i] = states_[i]->get();
    commands_[i]->set(hold_[i]);
  }
}

bool JointTrajectoryController::set_trajectory(
    const msgs::control::FollowJointTrajectoryGoal& goal, std::string& error) {
  const auto& message = goal.trajectory;
  if (message.points.empty() || message.joint_names.empty()) {
    error = "빈 궤적";
    return false;
  }
  std::vector<std::size_t> source_index;
  source_index.reserve(joints_.size());
  for (const auto& joint : joints_) {
    const auto found = std::find(message.joint_names.begin(), message.joint_names.end(), joint);
    if (found == message.joint_names.end()) {
      error = "관절이 궤적에 없다: " + joint;
      return false;
    }
    source_index.push_back(static_cast<std::size_t>(std::distance(message.joint_names.begin(), found)));
  }
  Trajectory trajectory;
  trajectory.times.reserve(message.points.size());
  trajectory.positions.assign(joints_.size(), {});
  double previous = 0.0;
  for (const auto& point : message.points) {
    if (point.positions.size() != message.joint_names.size()) {
      error = "JointTrajectoryPoint.positions 크기가 joint_names와 다르다";
      return false;
    }
    if (point.time_from_start < previous - 1e-9) {
      error = "time_from_start가 뒤로 간다";
      return false;
    }
    previous = point.time_from_start;
    trajectory.times.push_back(point.time_from_start);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      trajectory.positions[i].push_back(point.positions[source_index[i]]);
    }
  }
  trajectory.duration = trajectory.times.empty() ? 0.0 : trajectory.times.back();
  std::lock_guard<std::mutex> lock(mutex_);
  trajectory_ = std::move(trajectory);
  start_time_ = context().node->now();
  done_.store(false);
  progress_.store(0.0);
  return true;
}

std::vector<double> JointTrajectoryController::sample(double elapsed) const {
  std::vector<double> position = actual_;
  if (trajectory_.positions.empty() || trajectory_.times.empty()) {
    return position;
  }
  const auto& times = trajectory_.times;
  if (elapsed <= times.front()) {
    return trajectory_.positions[0];
  }
  if (elapsed >= times.back()) {
    std::vector<double> last;
    last.reserve(trajectory_.positions.size());
    for (const auto& joint : trajectory_.positions) {
      last.push_back(joint.back());
    }
    return last;
  }
  std::size_t index = 0;
  while (index + 1 < times.size() && times[index + 1] <= elapsed) {
    ++index;
  }
  const double segment = times[index + 1] - times[index];
  const double ratio = segment > 1e-9 ? std::clamp((elapsed - times[index]) / segment, 0.0, 1.0) : 1.0;
  std::vector<double> result;
  result.reserve(trajectory_.positions.size());
  for (const auto& joint : trajectory_.positions) {
    result.push_back(joint[index] + (joint[index + 1] - joint[index]) * ratio);
  }
  return result;
}

void JointTrajectoryController::update(double time, double period) {
  (void)period;
  std::vector<double> desired;
  bool finished = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < states_.size(); ++i) {
      actual_[i] = states_[i]->get();
    }
    if (!active_.load()) {
      publish_state(time, desired_, actual_, false);
      return;
    }
    if (!running_.load()) {
      // 궤적이 없을 때는 hold_ 위치를 유지한다. 측정값(actual)을 계속 목표로 넣으면
      // 중력으로 처진 위치가 곧 새 목표가 되어 조금씩 내려앉는다(래칫).
      // hold_는 활성화/취소 시 현재 위치, 궤적 완료 시 마지막 궤적 목표로 갱신된다.
      if (hold_.size() != commands_.size()) {
        hold_.assign(actual_.begin(), actual_.begin() + static_cast<std::ptrdiff_t>(
                                                      std::min(actual_.size(), commands_.size())));
      }
      for (std::size_t i = 0; i < commands_.size() && i < hold_.size(); ++i) {
        commands_[i]->set(hold_[i]);
      }
      desired_ = hold_;
      publish_state(time, desired_, actual_, false);
      return;
    }
    const double elapsed = time - start_time_;
    desired = sample(elapsed);
    desired_ = desired;
    for (std::size_t i = 0; i < commands_.size() && i < desired.size(); ++i) {
      commands_[i]->set(desired[i]);
    }
    progress_.store(trajectory_.duration > 1e-9 ? std::clamp(elapsed / trajectory_.duration, 0.0, 1.0)
                                                : 1.0);
    finished = elapsed >= trajectory_.duration + goal_time_tolerance_.value();
    if (finished) {
      running_.store(false);
      done_.store(true);
      // 완료 후에는 "궤적의 마지막 목표"를 유지한다 (측정값 아님).
      hold_.resize(commands_.size());
      for (std::size_t i = 0; i < hold_.size() && i < trajectory_.positions.size(); ++i) {
        if (!trajectory_.positions[i].empty()) {
          hold_[i] = trajectory_.positions[i].back();
        }
      }
    }
  }
  publish_state(time, desired, actual_, finished);
}

void JointTrajectoryController::publish_state(double time, const std::vector<double>& desired,
                                              const std::vector<double>& actual, bool force) {
  if (!force && time - last_state_time_ < 0.1) {
    return;
  }
  last_state_time_ = time;
  msgs::control::JointTrajectoryControllerState message;
  message.header.stamp = time;
  message.header.frame_id.clear();
  message.joint_names = joints_;
  message.desired.positions = desired;
  message.actual.positions = actual;
  message.error.positions.resize(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double target = i < desired.size() ? desired[i] : (i < actual.size() ? actual[i] : 0.0);
    const double current = i < actual.size() ? actual[i] : 0.0;
    message.error.positions[i] = target - current;
  }
  context().node->publish(state_topic_.value(), std::move(message), QOS_DEFAULT);
}

msgs::control::FollowJointTrajectoryResult JointTrajectoryController::execute(Handle& handle) {
  msgs::control::FollowJointTrajectoryResult result;
  std::string error;
  if (!set_trajectory(handle.goal(), error)) {
    result.error_code = msgs::control::FollowJointTrajectoryResult::INVALID_TRAJECTORY;
    result.error_string = error;
    context().node->logger().warn("joint_trajectory_controller: %s", error.c_str());
    return result;
  }
  running_.store(true);
  context().node->logger().info("joint_trajectory_controller: execute %zu points, %.2fs",
                                handle.goal().trajectory.points.size(),
                                handle.goal().trajectory.points.empty()
                                    ? 0.0
                                    : handle.goal().trajectory.points.back().time_from_start);
  double last_feedback = -1e18;
  while (!done_.load()) {
    if (handle.is_cancel_requested()) {
      running_.store(false);
      done_.store(true);
      apply_hold();
      result.error_code = msgs::control::FollowJointTrajectoryResult::SUCCESSFUL;
      result.error_string = "canceled";
      return result;
    }
    const double current = context().node->now();
    if (current - last_feedback >= 0.1) {
      last_feedback = current;
      msgs::control::FollowJointTrajectoryFeedback feedback;
      feedback.header.stamp = current;
      feedback.joint_names = joints_;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        feedback.desired.positions = desired_;
        feedback.actual.positions = actual_;
        feedback.error.positions.resize(joints_.size());
        for (std::size_t i = 0; i < joints_.size(); ++i) {
          const double target = i < desired_.size() ? desired_[i] : actual_[i];
          feedback.error.positions[i] = target - actual_[i];
        }
      }
      handle.publish_feedback(feedback);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  result.error_code = msgs::control::FollowJointTrajectoryResult::SUCCESSFUL;
  return result;
}

}  // namespace krosc::ros2_control
