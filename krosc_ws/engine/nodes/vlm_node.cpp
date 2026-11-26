#include "vlm_node.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"

#include "http_client.hpp"
#include "llm_json.hpp"

namespace krosc_ws {

namespace {

std::string quoted(const std::string& value) {
  return "\"" + llm::escape_json(value) + "\"";
}

std::string base64_encode(const std::vector<std::uint8_t>& data) {
  static constexpr char kTable[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((data.size() + 2U) / 3U * 4U);
  for (std::size_t index = 0; index < data.size(); index += 3U) {
    const unsigned first = data[index];
    const unsigned second = index + 1U < data.size() ? data[index + 1U] : 0U;
    const unsigned third = index + 2U < data.size() ? data[index + 2U] : 0U;
    out.push_back(kTable[first >> 2U]);
    out.push_back(kTable[((first & 0x03U) << 4U) | (second >> 4U)]);
    out.push_back(index + 1U < data.size()
                      ? kTable[((second & 0x0FU) << 2U) | (third >> 6U)]
                      : '=');
    out.push_back(index + 2U < data.size() ? kTable[third & 0x3FU] : '=');
  }
  return out;
}

}  // namespace

VlmNode::VlmNode(krosc::Context& context, std::string url, std::string camera_topic,
                 double timeout_s)
    : Node(context, "vlm"),
      url_(std::move(url)),
      camera_topic_(std::move(camera_topic)),
      timeout_s_(timeout_s > 0.0 ? timeout_s : 60.0) {
  camera_subscription_ = subscribe<krosc::msgs::Image>(
      camera_topic_, [this](const krosc::msgs::Image& message) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        image_ = message;
        image_seen_ = true;
      });
  create_service<krosc::msgs::String, krosc::msgs::String>(
      "vlm/query", [this](const krosc::msgs::String& request) {
        return handle_query(request);
      });
  logger().info("VlmNode: ready service=vlm/query sidecar=%s", url_.c_str());
}

krosc::msgs::String VlmNode::handle_query(const krosc::msgs::String& request) {
  std::string question = llm::string_field(request.data, "question").value_or("");
  if (question.empty()) {
    question = request.data.empty() ? "What do you see?" : request.data;
  }
  krosc::msgs::Image image;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!image_seen_) {
      return krosc::msgs::String{"VLM 오류: 카메라 프레임이 없다"};
    }
    image = image_;
  }
  const std::string body =
      "{\"image_b64\":\"" + base64_encode(image.data) + "\",\"encoding\":" +
      quoted(image.encoding) + ",\"width\":" + std::to_string(image.width) +
      ",\"height\":" + std::to_string(image.height) + ",\"question\":" +
      quoted(question) + "}";
  const HttpResponse response = http_post_json(url_, body, timeout_s_);
  if (!response.ok) {
    return krosc::msgs::String{"VLM 오류: " + response.error};
  }
  const auto answer = llm::string_field(response.body, "answer");
  if (!answer) {
    return krosc::msgs::String{"VLM 오류: 응답 파싱 실패"};
  }
  return krosc::msgs::String{*answer};
}

}  // namespace krosc_ws
