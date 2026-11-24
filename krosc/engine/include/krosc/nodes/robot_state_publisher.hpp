#pragma once

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/tf_ros.hpp"
#include "krosc/topic.hpp"
#include "krosc/urdf.hpp"

namespace krosc::nodes {

// URDF로 로봇 내부의 TF를 만든다 (ROS robot_state_publisher).
// robot_description 파라미터를 읽어 robot_description 토픽(latch), 고정 관절(tf_static),
// 움직이는 관절(joint_states -> tf)을 낸다.
class RobotStatePublisher final : public Node {
 public:
  explicit RobotStatePublisher(Context& context, std::string name = "robot_state_publisher",
                               std::map<std::string, ParamValue> parameters = {});

  [[nodiscard]] const UrdfRobot* robot() const { return robot_ ? &*robot_ : nullptr; }

 private:
  void rebuild(const std::string& text);
  void on_joint_states(const msgs::JointState& message);

  Parameter<std::string> robot_description_;
  std::optional<UrdfRobot> robot_;
  std::unordered_map<std::string, UrdfJoint> movable_;
  std::unordered_set<std::string> warned_unknown_;
  TransformBroadcaster tf_;
  StaticTransformBroadcaster static_tf_;
  Subscription<msgs::JointState> subscription_;
};

}  // namespace krosc::nodes
