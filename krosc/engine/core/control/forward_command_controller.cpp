#include "krosc/control/forward_command_controller.hpp"

#include <algorithm>
#include <utility>

#include "krosc/qos.hpp"

namespace krosc::control {

bool ForwardCommandController::on_init(ControllerContext& context) {
  if (!ControllerInterface::on_init(context)) {
    return false;
  }
  const auto configured = context.node->declare_parameter<std::vector<std::string>>(
      context.name + ".joints", std::vector<std::string>{});
  if (configured.value().empty()) {
    context.node->logger().error("forward_command_controller: %s.joints가 비어 있다",
                                 context.name.c_str());
    return false;
  }
  for (const auto& joint : configured.value()) {
    auto command = context.command(joint + "/position");
    if (!command) {
      context.node->logger().error("forward_command_controller: command interface가 없다: %s/position",
                                   joint.c_str());
      return false;
    }
    joints_.push_back(Joint{joint, std::move(command)});
  }
  command_topic_ = context.node->declare_parameter<std::string>(
      context.name + ".command_topic", "arm_joint_command");
  subscription_ = context.node->subscribe<msgs::JointState>(
      command_topic_.value(), [this](const msgs::JointState& message) { on_command(message); },
      QOS_DEFAULT);
  return true;
}

bool ForwardCommandController::on_activate() {
  std::lock_guard<std::mutex> lock(mutex_);
  has_pending_ = false;
  pending_.assign(joints_.size(), 0.0);
  return true;
}

void ForwardCommandController::on_command(const msgs::JointState& message) {
  std::vector<double> values(joints_.size(), 0.0);
  bool any = false;
  if (message.name.empty()) {
    for (std::size_t i = 0; i < std::min(joints_.size(), message.position.size()); ++i) {
      values[i] = message.position[i];
      any = true;
    }
  } else {
    for (std::size_t i = 0; i < message.name.size() && i < message.position.size(); ++i) {
      for (std::size_t joint = 0; joint < joints_.size(); ++joint) {
        if (joints_[joint].name == message.name[i]) {
          values[joint] = message.position[i];
          any = true;
          break;
        }
      }
    }
  }
  if (!any) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  pending_ = std::move(values);
  has_pending_ = true;
}

void ForwardCommandController::update(double time, double period) {
  (void)time;
  (void)period;
  std::vector<double> values;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_pending_) {
      return;
    }
    values = pending_;
    has_pending_ = false;
  }
  for (std::size_t i = 0; i < joints_.size() && i < values.size(); ++i) {
    joints_[i].command->set(values[i]);
  }
}

std::vector<std::string> ForwardCommandController::command_interface_names() const {
  std::vector<std::string> names;
  names.reserve(joints_.size());
  for (const auto& joint : joints_) {
    names.push_back(joint.name + "/position");
  }
  return names;
}

}  // namespace krosc::control
