#include "krosc/tf/tf_ros.hpp"

#include <utility>

namespace krosc {

TransformListener::TransformListener(Buffer& buffer, Node& node) : buffer_(buffer), node_(node) {
  tf_subscription_ = node_.subscribe<msgs::TFMessage>(
      TF_TOPIC_NAME, [this](const msgs::TFMessage& message) { on_message(message, false); }, QOS_TF);
  static_subscription_ = node_.subscribe<msgs::TFMessage>(
      TF_STATIC_TOPIC_NAME, [this](const msgs::TFMessage& message) { on_message(message, true); }, QOS_TF_STATIC);
}

TransformListener::~TransformListener() {
  tf_subscription_.cancel();
  static_subscription_.cancel();
}

void TransformListener::on_message(const msgs::TFMessage& message, bool is_static) {
  for (const auto& stamped : message.transforms) {
    try {
      buffer_.set_transform(stamped, is_static);
    } catch (const TransformException& error) {  // 잘못된 변환 하나가 나머지를 막으면 안 된다
      node_.logger().warn("TF를 버퍼에 넣지 못했다 (%s -> %s): %s", stamped.header.frame_id.c_str(),
                          stamped.child_frame_id.c_str(), error.what());
    } catch (const std::exception& error) {
      node_.logger().warn("TF를 버퍼에 넣지 못했다 (%s -> %s): %s", stamped.header.frame_id.c_str(),
                          stamped.child_frame_id.c_str(), error.what());
    }
  }
}

void TransformBroadcaster::send_transform(const msgs::TransformStamped& transform) {
  msgs::TFMessage message;
  message.transforms.push_back(transform);
  node_.publish(TF_TOPIC_NAME, std::move(message), QOS_TF);
}

void TransformBroadcaster::send_transform(const std::vector<msgs::TransformStamped>& transforms) {
  if (transforms.empty()) {
    return;
  }
  msgs::TFMessage message;
  message.transforms = transforms;
  node_.publish(TF_TOPIC_NAME, std::move(message), QOS_TF);
}

void StaticTransformBroadcaster::send_transform(const msgs::TransformStamped& transform) {
  msgs::TFMessage message;
  message.transforms.push_back(transform);
  node_.publish(TF_STATIC_TOPIC_NAME, std::move(message), QOS_TF_STATIC);
}

void StaticTransformBroadcaster::send_transform(const std::vector<msgs::TransformStamped>& transforms) {
  if (transforms.empty()) {
    return;
  }
  msgs::TFMessage message;
  message.transforms = transforms;
  node_.publish(TF_STATIC_TOPIC_NAME, std::move(message), QOS_TF_STATIC);
}

}  // namespace krosc
