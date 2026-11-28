#include "llm_client.hpp"

#include <string>
#include <utility>

#include "nodes/common/http_client.hpp"
#include "nodes/common/llm_json.hpp"

namespace srf_default_ws {

OllamaClient::OllamaClient(std::string base_url, std::string model, double timeout_s)
    : base_url_(std::move(base_url)),
      model_(std::move(model)),
      timeout_s_(timeout_s > 0.0 ? timeout_s : 10.0) {}

OllamaReply OllamaClient::chat(const std::string& system, const std::string& user,
                               const std::string& format_schema,
                               const std::string& image_b64) const {
  OllamaReply reply;
  std::string user_message =
      "{\"role\":\"user\",\"content\":\"" + llm::escape_json(user) + "\"";
  if (!image_b64.empty()) {
    user_message += ",\"images\":[\"" + image_b64 + "\"]";
  }
  user_message += "}";
  const std::string body =
      "{\"model\":\"" + llm::escape_json(model_) +
      "\",\"stream\":false,\"format\":" + format_schema +
      ",\"options\":{\"temperature\":0,\"num_predict\":160},\"messages\":["
      "{\"role\":\"system\",\"content\":\"" + llm::escape_json(system) + "\"}," +
      user_message + "]}";
  std::string url = base_url_;
  while (!url.empty() && url.back() == '/') {
    url.pop_back();
  }
  url += "/api/chat";
  const HttpResponse http = http_post_json(url, body, timeout_s_);
  reply.http_status = http.status;
  if (!http.ok) {
    reply.error = http.error;
    return reply;
  }
  auto content = llm::ollama_content(http.body);
  if (!content) {
    reply.error = "Ollama 응답에 message.content가 없다: " + http.body.substr(0, 200);
    return reply;
  }
  reply.ok = true;
  reply.content = std::move(*content);
  return reply;
}

}  // namespace srf_default_ws
