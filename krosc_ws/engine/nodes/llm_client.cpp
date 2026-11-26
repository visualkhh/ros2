#include "llm_client.hpp"

#include <string>
#include <utility>

#include "http_client.hpp"
#include "llm_json.hpp"

namespace krosc_ws {

OllamaClient::OllamaClient(std::string base_url, std::string model, double timeout_s)
    : base_url_(std::move(base_url)),
      model_(std::move(model)),
      timeout_s_(timeout_s > 0.0 ? timeout_s : 10.0) {}

OllamaReply OllamaClient::chat(const std::string& system, const std::string& user,
                               const std::string& format_schema) const {
  OllamaReply reply;
  const std::string body =
      "{\"model\":\"" + llm::escape_json(model_) +
      "\",\"stream\":false,\"format\":" + format_schema +
      ",\"options\":{\"temperature\":0,\"num_predict\":160},\"messages\":["
      "{\"role\":\"system\",\"content\":\"" + llm::escape_json(system) + "\"},"
      "{\"role\":\"user\",\"content\":\"" + llm::escape_json(user) + "\"}]}";
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

}  // namespace krosc_ws
