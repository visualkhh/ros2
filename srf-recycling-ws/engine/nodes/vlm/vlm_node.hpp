#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "srf/msgs.hpp"
#include "srf/node.hpp"
#include "srf/tf/tf.hpp"
#include "srf/tf/tf_ros.hpp"

namespace srf_recycling_ws {

// 최신 카메라 프레임을 SmolVLM2 sidecar(HTTP, ml/vlm/serve.py)에 보내 묻는 노드.
// service: vlm/query  (String JSON {"question":"..."}) -> 답 문자열
//          vlm/select (String JSON {"target":"can|pet|paper|all"}) -> String JSON
//            {"decision":"select|hold","reason":"ok|ambiguous|absent",
//             "target":{"category","label","bbox":[0~1000]}, "target_xyz":[x,y,z] (base_link, m), "answer":원출력}
//            학습 때(ml/vlm/vqa.py)와 같은 질문 문장을 쓴다. 3D 는 bbox 안 깊이(가까운 면) + TF.
struct VlmCamera {
  std::string color_frame = "camera_color_optical_frame";
  std::string depth_topic = "camera/depth/image_rect_raw";
  double color_hfov_deg = 69.0;  // D435 color
  double depth_hfov_deg = 87.0;  // D435 depth (시뮬레이터는 color 와 같은 렌더라 color 화각)
  double depth_vfov_deg = 58.0;
};
class VlmNode final : public srf::Node {
 public:
  VlmNode(srf::Context& context, std::string url, std::string camera_topic,
          double timeout_s, VlmCamera camera = {});
  ~VlmNode() override = default;

  VlmNode(const VlmNode&) = delete;
  VlmNode& operator=(const VlmNode&) = delete;

 private:
  srf::msgs::String handle_query(const srf::msgs::String& request);
  srf::msgs::String handle_select(const srf::msgs::String& request);
  // sidecar 에 묻는다. 실패면 nullopt + error
  std::optional<std::string> ask(const std::string& question, srf::msgs::Image& image, std::string& error);
  // bbox(0~1000) → base_link 3D. 깊이·TF 가 없으면 nullopt
  std::optional<srf::msgs::Vector3> locate(const srf::msgs::Image& color, const std::array<double, 4>& bbox);

  std::string url_;
  VlmCamera camera_;
  srf::Buffer tf_buffer_;
  std::unique_ptr<srf::TransformListener> tf_listener_;
  std::mutex depth_mutex_;
  srf::msgs::Image depth_;
  bool depth_seen_ = false;
  srf::Subscription<srf::msgs::Image> depth_subscription_;
  std::string camera_topic_;
  double timeout_s_ = 60.0;

  std::mutex image_mutex_;
  srf::msgs::Image image_;
  bool image_seen_ = false;

  std::mutex context_mutex_;
  std::string robot_context_;

  srf::Subscription<srf::msgs::String> context_subscription_;
  srf::Subscription<srf::msgs::Image> camera_subscription_;
};

}  // namespace srf_recycling_ws
