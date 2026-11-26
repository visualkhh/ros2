#include "opencr_control.hpp"

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

OpenCrControl::OpenCrControl(OpenCrDevice& opencr, double wheel_radius, double wheel_separation)
    : opencr_(opencr), wheel_radius_(wheel_radius), wheel_separation_(wheel_separation) {
  for (const auto& name :
       {"wheel_left_joint", "wheel_right_joint", "joint1", "joint2", "joint3", "joint4",
        "gripper_left_joint", "gripper_right_joint"}) {
    states_[name] = std::make_shared<krosc::control::StateInterface>(std::string(name) + "/position");
  }
  states_["wheel_left_joint/velocity"] =
      std::make_shared<krosc::control::StateInterface>("wheel_left_joint/velocity");
  states_["wheel_right_joint/velocity"] =
      std::make_shared<krosc::control::StateInterface>("wheel_right_joint/velocity");
  for (const auto& name : arm_joints()) {
    commands_[name] = std::make_shared<krosc::control::CommandInterface>(name + "/position");
  }
  commands_["wheel_left_joint/velocity"] =
      std::make_shared<krosc::control::CommandInterface>("wheel_left_joint/velocity");
  commands_["wheel_right_joint/velocity"] =
      std::make_shared<krosc::control::CommandInterface>("wheel_right_joint/velocity");
}

bool OpenCrControl::on_init(const krosc::control::ControlInfo& info) {
  if (!krosc::control::Control::on_init(info)) {
    return false;
  }
  last_write_time_ = -1e18;
  return true;
}

std::vector<std::shared_ptr<krosc::control::StateInterface>>
OpenCrControl::export_state_interfaces() {
  std::vector<std::shared_ptr<krosc::control::StateInterface>> result;
  result.reserve(states_.size());
  for (const auto& [name, state] : states_) {
    (void)name;
    result.push_back(state);
  }
  return result;
}

std::vector<std::shared_ptr<krosc::control::CommandInterface>>
OpenCrControl::export_command_interfaces() {
  std::vector<std::shared_ptr<krosc::control::CommandInterface>> result;
  result.reserve(commands_.size());
  for (const auto& [name, command] : commands_) {
    (void)name;
    result.push_back(command);
  }
  return result;
}

bool OpenCrControl::read(double time, double period) {
  (void)time;
  (void)period;
  states_["wheel_left_joint/velocity"]->set(opencr_.left_velocity());
  states_["wheel_right_joint/velocity"]->set(opencr_.right_velocity());
  states_["wheel_left_joint"]->set(opencr_.left_position());
  states_["wheel_right_joint"]->set(opencr_.right_position());

  const ArmStatus arm = opencr_.arm_status();
  if (arm.present) {
    for (std::size_t i = 0; i < 4; ++i) {
      if (!std::isfinite(arm.position[i])) {
        continue;
      }
      const auto found = states_.find("joint" + std::to_string(i + 1));
      if (found != states_.end()) {
        found->second->set(arm.position[i]);
      }
      arm_ready_ = true;
    }
    if (std::isfinite(arm.position[4])) {
      states_["gripper_left_joint"]->set(arm.position[4]);
      states_["gripper_right_joint"]->set(arm.position[4]);
    }
  }
  return true;
}

bool OpenCrControl::write(double time, double period) {
  (void)period;
  const auto left_velocity = commands_.find("wheel_left_joint/velocity");
  const auto right_velocity = commands_.find("wheel_right_joint/velocity");
  if (left_velocity != commands_.end() && right_velocity != commands_.end()) {
    krosc::msgs::Twist twist;
    twist.linear.x =
        (right_velocity->second->get() + left_velocity->second->get()) * wheel_radius_ / 2.0;
    twist.angular.z = (right_velocity->second->get() - left_velocity->second->get()) *
                      wheel_radius_ / wheel_separation_;
    opencr_.write(twist);
  }
  if (arm_ready_ && time - last_write_time_ >= 0.05) {
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
    if (any) {
      opencr_.write_arm(target);
    }
  }
  // 바퀴 C 명령/팔 keepalive는 여기서 한 번만 보낸다.
  opencr_.tick();
  return true;
}

}  // namespace krosc_ws
