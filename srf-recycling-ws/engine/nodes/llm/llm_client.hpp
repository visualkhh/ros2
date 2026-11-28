#pragma once

#include <string>

namespace srf_recycling_ws {

struct OllamaReply {
  bool ok = false;
  std::string content;  // message.content (모델이 만든 JSON 문자열)
  std::string error;
  int http_status = 0;
};

// Ollama /api/chat 전용 최소 HTTP 클라이언트. 외부 라이브러리 없이 POSIX 소켓만 쓴다.
// 서버 IP/모델은 생성자에서 받고, 요청은 워커 스레드에서 호출한다.
class OllamaClient {
 public:
  OllamaClient(std::string base_url, std::string model, double timeout_s);

  OllamaReply chat(const std::string& system, const std::string& user,
                   const std::string& format_schema,
                   const std::string& image_b64 = {}) const;

 private:
  std::string base_url_;
  std::string model_;
  double timeout_s_ = 10.0;
};

}  // namespace srf_recycling_ws
