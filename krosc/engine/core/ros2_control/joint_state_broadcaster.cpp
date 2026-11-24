#include "krosc/ros2_control/joint_state_broadcaster.hpp"

#include <utility>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"

namespace krosc::ros2_control {

bool JointStateBroadcaster::on_init(ControllerContext& context) {
  if (!ControllerInterface::on_init(context)) {
    return false;
  }
  joints_parameter_ = context.node->declare_parameter<std::vector<std::string>>(
      context.name + ".joints", std::vector<std::string>{});
  const auto configured = joints_parameter_.value();
  if (!configured.empty()) {
    for (const auto& joint : configured) {
      auto state = context.state(joint + "/position");
      if (!state) {
        context.node->logger().error("joint_state_broadcaster: state interface가 없다: %s/position",
                                     joint.c_str());
        return false;
      }
      joints_.emplace_back(joint, std::move(state));
    }
    return true;
  }
  // joints를 비우면 하드웨어가 내보낸 모든 position interface를 발행한다 (JSB 기본 동작과 같다).
  for (const auto& [interface_name, state] : context.state_interfaces) {
    const std::string suffix = "/position";
    if (interface_name.size() <= suffix.size() ||
        interface_name.compare(interface_name.size() - suffix.size(), suffix.size(), suffix) != 0) {
      continue;
    }
    joints_.emplace_back(interface_name.substr(0, interface_name.size() - suffix.size()), state);
  }
  return !joints_.empty();
}

bool JointStateBroadcaster::on_activate() { return !joints_.empty(); }

void JointStateBroadcaster::update(double time, double period) {
  (void)period;
  if (joints_.empty()) {
    return;
  }
  msgs::JointState message;
  message.header.stamp = time;
  message.name.reserve(joints_.size());
  message.position.reserve(joints_.size());
  for (const auto& [joint, state] : joints_) {
    message.name.push_back(joint);
    message.position.push_back(state->get());
  }
  context().node->publish("joint_states", std::move(message), QOS_DEFAULT);
}

}  // namespace krosc::ros2_control
