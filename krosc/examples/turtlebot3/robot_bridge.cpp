#include "robot_bridge.hpp"

#include <utility>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf_ros.hpp"

namespace krosc::examples {

RobotBridge::RobotBridge(krosc::Context& context, krosc::sim::TurtleBot3& robot)
    : Node(context, "RobotBridge"), robot_(robot) {
  robot_.set_scan_callback(
      [this](const krosc::msgs::LaserScan& scan) { publish("scan", scan, krosc::QOS_SENSOR_DATA); });
  robot_.set_imu_callback([this](const krosc::msgs::Imu& imu) { publish("imu", imu, krosc::QOS_SENSOR_DATA); });
  subscribe<krosc::msgs::Twist>(
      "cmd_vel", [this](const krosc::msgs::Twist& twist) { robot_.set_command(twist); }, krosc::QOS_COMMAND);
  joint_states_timer_ = create_timer(0.05, [this] { publish_joint_states(); });
}

void RobotBridge::publish_joint_states() {
  const auto positions = robot_.joint_positions();
  if (positions.empty()) {
    return;
  }
  krosc::msgs::JointState state;
  state.header.stamp = now();
  state.name.reserve(positions.size());
  state.position.reserve(positions.size());
  for (const auto& [name, position] : positions) {
    state.name.push_back(name);
    state.position.push_back(position);
  }
  publish(krosc::JOINT_STATES_TOPIC_NAME, std::move(state));
}

}  // namespace krosc::examples
