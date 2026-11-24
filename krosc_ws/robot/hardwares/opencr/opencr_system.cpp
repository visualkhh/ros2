#include "opencr_system.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace krosc_ws {

namespace {
const std::vector<std::string>& arm_joints() {
  static const std::vector<std::string> joints{"joint1", "joint2", "joint3", "joint4",
                                               "gripper_left_joint"};
  return joints;
}
}  // namespace

OpenCrSystem::OpenCrSystem(RealTurtleBot3& robot) : robot_(robot) {
  for (const auto& name :
       {"wheel_left_joint", "wheel_right_joint", "joint1", "joint2", "joint3", "joint4",
        "gripper_left_joint", "gripper_right_joint"}) {
    states_[name] = std::make_shared<krosc::ros2_control::StateInterface>(std::string(name) + "/position");
  }
  states_["wheel_left_joint/velocity"] =
      std::make_shared<krosc::ros2_control::StateInterface>("wheel_left_joint/velocity");
  states_["wheel_right_joint/velocity"] =
      std::make_shared<krosc::ros2_control::StateInterface>("wheel_right_joint/velocity");
  for (const auto& name : arm_joints()) {
    commands_[name] = std::make_shared<krosc::ros2_control::CommandInterface>(name + "/position");
  }
  commands_["wheel_left_joint/velocity"] =
      std::make_shared<krosc::ros2_control::CommandInterface>("wheel_left_joint/velocity");
  commands_["wheel_right_joint/velocity"] =
      std::make_shared<krosc::ros2_control::CommandInterface>("wheel_right_joint/velocity");
}

bool OpenCrSystem::on_init(const krosc::ros2_control::HardwareInfo& info) {
  if (!krosc::ros2_control::SystemInterface::on_init(info)) {
    return false;
  }
  last_write_time_ = -1e18;
  return true;
}

std::vector<std::shared_ptr<krosc::ros2_control::StateInterface>>
OpenCrSystem::export_state_interfaces() {
  std::vector<std::shared_ptr<krosc::ros2_control::StateInterface>> result;
  result.reserve(states_.size());
  for (const auto& [name, state] : states_) {
    (void)name;
    result.push_back(state);
  }
  return result;
}

std::vector<std::shared_ptr<krosc::ros2_control::CommandInterface>>
OpenCrSystem::export_command_interfaces() {
  std::vector<std::shared_ptr<krosc::ros2_control::CommandInterface>> result;
  result.reserve(commands_.size());
  for (const auto& [name, command] : commands_) {
    (void)name;
    result.push_back(command);
  }
  return result;
}

bool OpenCrSystem::read(double time, double period) {
  (void)time;
  (void)period;
  states_["wheel_left_joint/velocity"]->set(robot_.opencr().left_velocity());
  states_["wheel_right_joint/velocity"]->set(robot_.opencr().right_velocity());
  const auto positions = robot_.joint_positions();
  for (const auto& [name, position] : positions) {
    const auto found = states_.find(name);
    if (found != states_.end()) {
      found->second->set(position);
    }
    if ((name == "joint1" || name == "joint2" || name == "joint3" || name == "joint4") &&
        std::isfinite(position)) {
      arm_ready_ = true;
    }
  }
  return true;
}

bool OpenCrSystem::write(double time, double period) {
  (void)period;
  const auto left_velocity = commands_.find("wheel_left_joint/velocity");
  const auto right_velocity = commands_.find("wheel_right_joint/velocity");
  if (left_velocity != commands_.end() && right_velocity != commands_.end()) {
    krosc::msgs::Twist twist;
    twist.linear.x = (right_velocity->second->get() + left_velocity->second->get()) *
                     RealTurtleBot3::kWheelRadius / 2.0;
    twist.angular.z = (right_velocity->second->get() - left_velocity->second->get()) *
                      RealTurtleBot3::kWheelRadius / RealTurtleBot3::kWheelSeparation;
    robot_.opencr().write(twist);
  }
  if (!arm_ready_) {
    return true;  // OpenCR에서 팔 상태를 아직 못 받았다: 0을 쏘지 않는다
  }
  if (time - last_write_time_ < 0.05) {  // OpenCR 팔 명령은 20Hz면 충분하다
    return true;
  }
  last_write_time_ = time;
  std::array<double, 5> target{std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN()};
  bool any = false;
  for (std::size_t index = 0; index < arm_joints().size(); ++index) {
    const auto found = commands_.find(arm_joints()[index]);
    if (found == commands_.end()) {
      continue;
    }
    const double value = found->second->get();
    if (std::isfinite(value)) {
      target[index] = value;
      any = true;
    }
  }
  if (!any) {
    return true;
  }
  robot_.opencr().write_arm(target);
  return true;
}

}  // namespace krosc_ws
