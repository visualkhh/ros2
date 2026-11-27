#pragma once

#include <mutex>
#include <string>

#include "krosc/node.hpp"

namespace krosc_ws {

// 최신 카메라 프레임을 SmolVLM2 sidecar(HTTP)에 보내 질문에 답하는 노드.
// service: vlm/query (String JSON {"question":"..."}) -> answer string
class VlmNode final : public krosc::Node {
 public:
  VlmNode(krosc::Context& context, std::string url, std::string camera_topic,
          double timeout_s);
  ~VlmNode() override = default;

  VlmNode(const VlmNode&) = delete;
  VlmNode& operator=(const VlmNode&) = delete;

 private:
  krosc::msgs::String handle_query(const krosc::msgs::String& request);

  std::string url_;
  std::string camera_topic_;
  double timeout_s_ = 60.0;

  std::mutex image_mutex_;
  krosc::msgs::Image image_;
  bool image_seen_ = false;

  std::mutex context_mutex_;
  std::string robot_context_;

  krosc::Subscription<krosc::msgs::String> context_subscription_;
  krosc::Subscription<krosc::msgs::Image> camera_subscription_;
};

}  // namespace krosc_ws
