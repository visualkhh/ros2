#pragma once

#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf.hpp"
#include "krosc/topic.hpp"

namespace krosc {

inline constexpr const char* TF_TOPIC_NAME = "tf";
inline constexpr const char* TF_STATIC_TOPIC_NAME = "tf_static";
inline constexpr const char* JOINT_STATES_TOPIC_NAME = "joint_states";
inline constexpr const char* ROBOT_DESCRIPTION_TOPIC_NAME = "robot_description";

// tf / tf_static 토픽을 구독해 Buffer를 채운다. 조회는 사용자가 만든 Buffer에 한다 (tf2_ros와 같다).
class TransformListener {
 public:
  TransformListener(Buffer& buffer, Node& node);
  ~TransformListener();

  TransformListener(const TransformListener&) = delete;
  TransformListener& operator=(const TransformListener&) = delete;

 private:
  void on_message(const msgs::TFMessage& message, bool is_static);

  Buffer& buffer_;
  Node& node_;
  Subscription<msgs::TFMessage> tf_subscription_;
  Subscription<msgs::TFMessage> static_subscription_;
};

// 움직이는 변환을 tf 토픽으로 내보낸다. header.frame_id가 부모, child_frame_id가 자식이다.
class TransformBroadcaster {
 public:
  explicit TransformBroadcaster(Node& node) : node_(node) {}

  void send_transform(const msgs::TransformStamped& transform);
  void send_transform(const std::vector<msgs::TransformStamped>& transforms);
  void sendTransform(const msgs::TransformStamped& transform) { send_transform(transform); }
  void sendTransform(const std::vector<msgs::TransformStamped>& transforms) { send_transform(transforms); }

 private:
  Node& node_;
};

// 안 변하는 변환을 tf_static에 한 번 내보낸다 (latch라 늦게 뜬 리스너도 받는다).
class StaticTransformBroadcaster {
 public:
  explicit StaticTransformBroadcaster(Node& node) : node_(node) {}

  void send_transform(const msgs::TransformStamped& transform);
  void send_transform(const std::vector<msgs::TransformStamped>& transforms);
  void sendTransform(const msgs::TransformStamped& transform) { send_transform(transform); }
  void sendTransform(const std::vector<msgs::TransformStamped>& transforms) { send_transform(transforms); }

 private:
  Node& node_;
};

}  // namespace krosc
