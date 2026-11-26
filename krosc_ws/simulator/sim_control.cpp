#include "sim_control.hpp"

#include <string>

namespace krosc_ws::sim {

SimControl::SimControl(krosc::simulator::TurtleBot3& robot, double wheel_radius,
                       double wheel_separation)
    : robot_(robot), wheel_radius_(wheel_radius), wheel_separation_(wheel_separation) {
  for (const auto& name : {"wheel_left_joint", "wheel_right_joint"}) {
    states_[name] = std::make_shared<krosc::control::StateInterface>(std::string(name) + "/position");
    commands_[name] = std::make_shared<krosc::control::CommandInterface>(std::string(name) + "/velocity");
  }
  states_["wheel_left_joint/velocity"] =
      std::make_shared<krosc::control::StateInterface>("wheel_left_joint/velocity");
  states_["wheel_right_joint/velocity"] =
      std::make_shared<krosc::control::StateInterface>("wheel_right_joint/velocity");
}

bool SimControl::read(double, double) {
  states_["wheel_left_joint"]->set(robot_.left_position());
  states_["wheel_right_joint"]->set(robot_.right_position());
  states_["wheel_left_joint/velocity"]->set(robot_.left_velocity());
  states_["wheel_right_joint/velocity"]->set(robot_.right_velocity());
  return true;
}

bool SimControl::write(double, double) {
  const double left = commands_["wheel_left_joint"]->get();
  const double right = commands_["wheel_right_joint"]->get();
  krosc::msgs::Twist twist;
  twist.linear.x = (right + left) * wheel_radius_ / 2.0;
  twist.angular.z = (right - left) * wheel_radius_ / wheel_separation_;
  robot_.set_command(twist);
  return true;
}

std::vector<std::shared_ptr<krosc::control::StateInterface>>
SimControl::export_state_interfaces() {
  std::vector<std::shared_ptr<krosc::control::StateInterface>> result;
  result.reserve(states_.size());
  for (const auto& [name, state] : states_) {
    (void)name;
    result.push_back(state);
  }
  return result;
}

std::vector<std::shared_ptr<krosc::control::CommandInterface>>
SimControl::export_command_interfaces() {
  std::vector<std::shared_ptr<krosc::control::CommandInterface>> result;
  result.reserve(commands_.size());
  for (const auto& [name, command] : commands_) {
    (void)name;
    result.push_back(command);
  }
  return result;
}

}  // namespace krosc_ws::sim
