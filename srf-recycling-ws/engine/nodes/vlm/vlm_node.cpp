#include "vlm_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "srf/msgs.hpp"
#include "srf/qos.hpp"
#include "srf/tf/math.hpp"

#include "nodes/common/http_client.hpp"
#include "nodes/common/llm_json.hpp"

namespace srf_recycling_ws {

namespace {

std::string json_quoted(const std::string& value) {
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

VlmNode::VlmNode(srf::Context& context, std::string url, std::string camera_topic,
                 double timeout_s, VlmCamera camera)
    : Node(context, "vlm"),
      url_(std::move(url)),
      camera_(std::move(camera)),
      camera_topic_(std::move(camera_topic)),
      timeout_s_(timeout_s > 0.0 ? timeout_s : 60.0) {
  context_subscription_ = subscribe<srf::msgs::String>(
      "robot_context", [this](const srf::msgs::String& message) {
        std::lock_guard<std::mutex> lock(context_mutex_);
        robot_context_ = message.data;
      });
  camera_subscription_ = subscribe<srf::msgs::Image>(
      camera_topic_, [this](const srf::msgs::Image& message) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        image_ = message;
        image_seen_ = true;
      });
  tf_listener_ = std::make_unique<srf::TransformListener>(tf_buffer_, *this);
  depth_subscription_ = subscribe<srf::msgs::Image>(
      camera_.depth_topic, [this](const srf::msgs::Image& message) {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        depth_ = message;
        depth_seen_ = true;
      });
  create_service<srf::msgs::String, srf::msgs::String>(
      "vlm/query", [this](const srf::msgs::String& request) {
        return handle_query(request);
      });
  create_service<srf::msgs::String, srf::msgs::String>(
      "vlm/select", [this](const srf::msgs::String& request) {
        return handle_select(request);
      });
  logger().info("VlmNode: ready service=vlm/query,vlm/select sidecar=%s", url_.c_str());
}

srf::msgs::String VlmNode::handle_query(const srf::msgs::String& request) {
  const auto publish_status = [this](const std::string& stage,
                                     const std::string& key,
                                     const std::string& value) {
    std::string json = "{\"stage\":\"" + stage + "\"";
    if (!key.empty()) {
      json += ",\"" + key + "\":\"" + llm::escape_json(value) + "\"";
    }
    json += "}";
    publish<srf::msgs::String>("vlm/status", srf::msgs::String{json},
                                 srf::QOS_DEFAULT);
  };
  std::string question = llm::string_field(request.data, "question").value_or("");
  if (question.empty()) {
    question = request.data.empty() ? "What do you see?" : request.data;
  }
  srf::msgs::Image image;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!image_seen_) {
      publish_status("error", "error", "카메라 프레임이 없다");
      return srf::msgs::String{"VLM 오류: 카메라 프레임이 없다"};
    }
    image = image_;
  }
  std::string robot_context;
  {
    std::lock_guard<std::mutex> lock(context_mutex_);
    robot_context = robot_context_;
  }
  publish_status("querying", "question", question);
  const std::string body =
      "{\"image_b64\":\"" + base64_encode(image.data) + "\",\"encoding\":" +
      json_quoted(image.encoding) + ",\"width\":" + std::to_string(image.width) +
      ",\"height\":" + std::to_string(image.height) + ",\"question\":" +
      json_quoted(question) + ",\"context\":" + json_quoted(robot_context) + "}";
  const HttpResponse response = http_post_json(url_, body, timeout_s_);
  if (!response.ok) {
    publish_status("error", "error", response.error);
    return srf::msgs::String{"VLM 오류: " + response.error};
  }
  const auto answer = llm::string_field(response.body, "answer");
  if (!answer) {
    publish_status("error", "error", "응답 파싱 실패");
    return srf::msgs::String{"VLM 오류: 응답 파싱 실패"};
  }
  publish_status("done", "answer", *answer);
  return srf::msgs::String{*answer};
}

std::optional<std::string> VlmNode::ask(const std::string& question, srf::msgs::Image& image, std::string& error) {
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!image_seen_) {
      error = "카메라 프레임이 없다";
      return std::nullopt;
    }
    image = image_;
  }
  const std::string body =
      "{\"image_b64\":\"" + base64_encode(image.data) + "\",\"encoding\":" + json_quoted(image.encoding) +
      ",\"width\":" + std::to_string(image.width) + ",\"height\":" + std::to_string(image.height) +
      ",\"question\":" + json_quoted(question) + "}";
  const HttpResponse response = http_post_json(url_, body, timeout_s_);
  if (!response.ok) {
    error = response.error;
    return std::nullopt;
  }
  auto answer = llm::string_field(response.body, "answer");
  if (!answer) {
    error = "응답 파싱 실패";
    return std::nullopt;
  }
  return answer;
}

std::optional<srf::msgs::Vector3> VlmNode::locate(const srf::msgs::Image& color, const std::array<double, 4>& bbox) {
  constexpr double kPi = 3.14159265358979323846;
  srf::msgs::Image depth;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (!depth_seen_) return std::nullopt;
    depth = depth_;
  }
  const int dw = static_cast<int>(depth.width), dh = static_cast<int>(depth.height);
  if (depth.encoding != "16UC1" || dw <= 0 || dh <= 0 || color.width == 0 || color.height == 0) return std::nullopt;
  const int step = depth.step > 0 ? static_cast<int>(depth.step) : dw * 2;
  const double cw = color.width, ch = color.height;
  const double c_fx = (cw * 0.5) / std::tan(camera_.color_hfov_deg * kPi / 360.0);
  const double d_fx = (dw * 0.5) / std::tan(camera_.depth_hfov_deg * kPi / 360.0);
  const double d_fy = (dh * 0.5) / std::tan(camera_.depth_vfov_deg * kPi / 360.0);
  // bbox 가운데 60% 의 깊이를 모아 가까운 면(20 백분위)을 물체 앞면으로 본다
  const double x0 = bbox[0] / 1000.0 * cw, y0 = bbox[1] / 1000.0 * ch, x1 = bbox[2] / 1000.0 * cw, y1 = bbox[3] / 1000.0 * ch;
  const double mx = (x1 - x0) * 0.2, my = (y1 - y0) * 0.2;
  std::vector<double> depths;
  for (double y = y0 + my; y <= y1 - my; y += 1.0) {
    for (double x = x0 + mx; x <= x1 - mx; x += 1.0) {
      const double tx = (x - cw * 0.5) / c_fx, ty = (y - ch * 0.5) / c_fx;  // color 광선 → depth 픽셀
      const int du = static_cast<int>(dw * 0.5 + d_fx * tx), dv = static_cast<int>(dh * 0.5 + d_fy * ty);
      if (du < 0 || du >= dw || dv < 0 || dv >= dh) continue;
      const std::size_t at = static_cast<std::size_t>(dv) * static_cast<std::size_t>(step) + static_cast<std::size_t>(du) * 2;
      const double meters = (depth.data[at] | (depth.data[at + 1] << 8)) / 1000.0;
      if (meters > 0.10 && meters < 2.5) depths.push_back(meters);
    }
  }
  if (depths.size() < 5) return std::nullopt;
  std::sort(depths.begin(), depths.end());
  const double surface = depths[depths.size() / 5];
  const double width = (x1 - x0) * surface / c_fx;
  const double z = surface + width * 0.5;  // 앞면 → 중심 (원기둥·상자 반 두께)
  const double u = (x0 + x1) * 0.5, v = (y0 + y1) * 0.5;
  srf::msgs::Vector3 point{(u - cw * 0.5) * z / c_fx, (v - ch * 0.5) * z / c_fx, z};
  try {
    const auto transform = tf_buffer_.lookup_transform("base_link", camera_.color_frame,
                                                       color.header.stamp > 0.0 ? color.header.stamp - 0.05 : now());
    return srf::tfmath::apply(transform.transform, point);
  } catch (const std::exception& exception) {
    logger().warn("VlmNode: TF base_link<-%s 없음: %s", camera_.color_frame.c_str(), exception.what());
    return std::nullopt;
  }
}

srf::msgs::String VlmNode::handle_select(const srf::msgs::String& request) {
  std::string target = llm::string_field(request.data, "target").value_or("all");
  static const std::map<std::string, std::string> kWords{{"can", "can"}, {"pet", "PET bottle"}, {"paper", "paper"}};
  const auto word = kWords.find(target);
  // ml/vlm/vqa.py 의 select 문장과 같게
  const std::string instruction = word == kWords.end() ? "sort all recyclables" : "sort the " + word->second;
  const std::string question = "Instruction: " + instruction + ". Choose one object to grab, or hold. Reply in JSON.";
  srf::msgs::Image image;
  std::string error;
  const auto answer = ask(question, image, error);
  if (!answer) {
    return srf::msgs::String{"{\"decision\":\"hold\",\"reason\":\"error\",\"error\":" + json_quoted(error) + "}"};
  }
  const std::string decision = llm::string_field(*answer, "decision").value_or("");
  const std::string reason = llm::string_field(*answer, "reason").value_or("low_confidence");
  if (decision != "select") {
    // 형식이 틀린 답도 보류로 (뒤 동작을 막는다)
    return srf::msgs::String{"{\"decision\":\"hold\",\"reason\":" + json_quoted(decision == "hold" ? reason : "invalid") +
                             ",\"answer\":" + json_quoted(*answer) + "}"};
  }
  const auto bbox_at = answer->find("\"bbox\"");
  std::array<double, 4> bbox{};
  bool box_ok = false;
  if (bbox_at != std::string::npos) {
    const auto open = answer->find('[', bbox_at);
    if (open != std::string::npos) {
      const char* p = answer->c_str() + open + 1;
      char* end = nullptr;
      int n = 0;
      for (; n < 4; ++n) {
        bbox[static_cast<std::size_t>(n)] = std::strtod(p, &end);
        if (end == p) break;
        p = end;
        while (*p == ',' || *p == ' ') ++p;
      }
      box_ok = n == 4 && bbox[2] > bbox[0] && bbox[3] > bbox[1];
    }
  }
  const std::string category = llm::string_field(*answer, "category").value_or("");
  const std::string label = llm::string_field(*answer, "label").value_or("");
  std::string json = "{\"decision\":\"select\",\"reason\":\"ok\",\"target\":{\"category\":" + json_quoted(category) +
                     ",\"label\":" + json_quoted(label);
  if (box_ok) {
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), ",\"bbox\":[%.0f,%.0f,%.0f,%.0f]", bbox[0], bbox[1], bbox[2], bbox[3]);
    json += buffer;
  }
  json += "}";
  if (box_ok) {
    if (const auto point = locate(image, bbox)) {
      char buffer[96];
      std::snprintf(buffer, sizeof(buffer), ",\"target_xyz\":[%.4f,%.4f,%.4f]", point->x, point->y, point->z);
      json += buffer;
    }
  }
  json += ",\"answer\":" + json_quoted(*answer) + "}";
  publish<srf::msgs::String>("vlm/status", srf::msgs::String{"{\"stage\":\"select\",\"result\":" + json + "}"},
                             srf::QOS_DEFAULT);
  return srf::msgs::String{json};
}

}  // namespace srf_recycling_ws
