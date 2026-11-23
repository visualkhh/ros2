#include "krosc/nodes/robot_state_publisher.hpp"

#include <cstdio>
#include <utility>
#include <vector>

namespace krosc::nodes {

namespace {

bool is_valid_urdf(const std::string& text) {
  if (text.empty()) {
    return true;  // 아직 로봇 설명이 없다
  }
  try {
    (void)parse_urdf(text);
    return true;
  } catch (const UrdfError& error) {
    std::fprintf(stderr, "[krosc] robot_description을 받아들이지 않는다: %s\n", error.what());
    return false;
  }
}

msgs::Transform origin(const UrdfJoint& joint) {
  msgs::Transform transform;
  transform.translation = {joint.xyz[0], joint.xyz[1], joint.xyz[2]};
  transform.rotation = tfmath::from_rpy(joint.rpy[0], joint.rpy[1], joint.rpy[2]);
  return transform;
}

}  // namespace

RobotStatePublisher::RobotStatePublisher(Context& context, std::string name,
                                         std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      robot_description_(declare_parameter<std::string>("robot_description", "",
                                                        [](const std::string& text) { return is_valid_urdf(text); },
                                                        [this](const std::string& text) { rebuild(text); })),
      tf_(*this),
      static_tf_(*this) {
  subscription_ = subscribe<msgs::JointState>(JOINT_STATES_TOPIC_NAME,
                                             [this](const msgs::JointState& message) { on_joint_states(message); });
  rebuild(robot_description_.value());
}

void RobotStatePublisher::rebuild(const std::string& text) {
  warned_unknown_.clear();
  if (text.empty()) {
    robot_.reset();
    movable_.clear();
    return;
  }
  try {
    robot_ = parse_urdf(text);
  } catch (const UrdfError& error) {
    logger().error("robot_description을 읽지 못했다: %s", error.what());
    robot_.reset();
    movable_.clear();
    return;
  }
  movable_.clear();
  for (const auto& joint : robot_->joints) {
    if (joint.movable()) {
      movable_.emplace(joint.name, joint);
    }
  }
  publish(ROBOT_DESCRIPTION_TOPIC_NAME, msgs::String{text}, QOS_LATCHED);
  const double stamp = now();
  std::vector<msgs::TransformStamped> fixed;
  for (const auto& joint : robot_->joints) {
    if (joint.type == "fixed") {
      fixed.push_back(msgs::TransformStamped{msgs::Header{stamp, joint.parent}, joint.child, origin(joint)});
    }
  }
  static_tf_.send_transform(fixed);
  for (const auto& joint : robot_->joints) {
    if (joint.type == "floating" || joint.type == "planar") {
      logger().warn("관절 %s의 type %s는 지원하지 않아 건너뛴다", joint.name.c_str(), joint.type.c_str());
    }
  }
  logger().info("로봇 %s: 고정 관절 %d개(tf_static), 움직이는 관절 %d개", robot_->name.c_str(),
                static_cast<int>(fixed.size()), static_cast<int>(movable_.size()));
}

void RobotStatePublisher::on_joint_states(const msgs::JointState& message) {
  if (message.name.size() != message.position.size()) {
    logger().error("joint_states의 name(%d개)과 position(%d개) 길이가 다르다",
                   static_cast<int>(message.name.size()), static_cast<int>(message.position.size()));
    return;
  }
  std::vector<msgs::TransformStamped> transforms;
  for (std::size_t i = 0; i < message.name.size(); ++i) {
    auto it = movable_.find(message.name[i]);
    if (it == movable_.end()) {
      if (warned_unknown_.insert(message.name[i]).second) {
        logger().debug("URDF에 움직이는 관절로 없는 이름을 무시한다: %s", message.name[i].c_str());
      }
      continue;
    }
    const UrdfJoint& joint = it->second;
    msgs::Transform motion;
    if (joint.type == "prismatic") {
      motion.translation = {joint.axis[0] * message.position[i], joint.axis[1] * message.position[i],
                            joint.axis[2] * message.position[i]};
    } else {
      motion.rotation = tfmath::from_axis_angle({joint.axis[0], joint.axis[1], joint.axis[2]}, message.position[i]);
    }
    transforms.push_back(msgs::TransformStamped{msgs::Header{message.header.stamp, joint.parent}, joint.child,
                                                tfmath::compose(origin(joint), motion)});
  }
  tf_.send_transform(transforms);
}

}  // namespace krosc::nodes
