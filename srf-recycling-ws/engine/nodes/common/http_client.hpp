#pragma once

#include <string>

namespace srf_recycling_ws {

struct HttpResponse {
  bool ok = false;
  int status = 0;
  std::string body;
  std::string error;
};

// 의존성 없는 HTTP/1.1 POST(JSON). http://만 지원한다.
// 로컬 Ollama / VLA / VLM sidecar 호출에 쓴다.
HttpResponse http_post_json(const std::string& url, const std::string& body,
                            double timeout_s);

// 의존성 없는 HTTP/1.1 GET. sidecar/Ollama health 확인용.
HttpResponse http_get(const std::string& url, double timeout_s);

}  // namespace srf_recycling_ws
