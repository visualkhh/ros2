#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "srf/msgs.hpp"
#include "srf/node.hpp"

#include "llm_client.hpp"

namespace srf_recycling_ws {

// 사람 말 → 할 일(intent + 파라미터). 해석만 하고 실행하지 않는다 (실행은 coordinator).
//   자주 쓰는 명령은 규칙(command_rules)으로 결정적으로, 나머지는 Ollama(/api/chat, JSON 스키마)로.
//   LLM 서버가 죽어도 규칙에 있는 명령은 계속 된다. 로봇 상태(robot_context)와 카메라 영상을 같이 준다.
// service: llm/parse (String 문장 -> String JSON {"ok","source","via","plan":{"intent",...}} | {"ok":false,"error"})
class Llm final : public srf::Node {
 public:
  Llm(srf::Context& context, std::string ollama_url, std::string model, double timeout_s,
      std::string mode = "llm", std::string camera_topic = "camera/color/image_raw", bool include_image = true);
  ~Llm() override;

  Llm(const Llm&) = delete;
  Llm& operator=(const Llm&) = delete;

  // 문장 하나를 해석한다 (서비스와 시험이 같이 쓴다).
  [[nodiscard]] std::string parse(const std::string& text);

 private:
  std::unique_ptr<OllamaClient> client_;
  std::string mode_;
  std::string camera_topic_;
  bool include_image_ = true;
  std::mutex context_mutex_;
  std::string robot_context_;
  std::mutex image_mutex_;
  srf::msgs::Image image_;
  bool image_seen_ = false;
  srf::Subscription<srf::msgs::String> context_subscription_;
  srf::Subscription<srf::msgs::Image> image_subscription_;
};

}  // namespace srf_recycling_ws
