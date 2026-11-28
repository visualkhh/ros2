#include "prompt_input.hpp"

#include <algorithm>
#include <any>
#include <cctype>

#include "nodes/common/llm_json.hpp"

namespace srf_recycling_ws {

namespace {

std::string trim(std::string text) {
  const auto not_space = [](unsigned char c) { return std::isspace(c) == 0; };
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
  text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
  return text;
}

std::string json_quoted(const std::string& value) { return "\"" + llm::escape_json(value) + "\""; }

bool is_stop(const std::string& text) {
  std::string lowered = text;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) { return std::tolower(c); });
  return lowered == "stop" || text == "정지" || text == "멈춰" || text == "그만";
}

}  // namespace

PromptInput::PromptInput(srf::Context& context)
    : Node(context, "prompt_input"), task_(std::make_unique<TaskClient>(context, "coordinator/task")) {
  create_service<srf::msgs::String, srf::msgs::String>(
      "prompt/submit", [this](const srf::msgs::String& request) { return submit(request); });
  logger().info("PromptInput: ready (service=prompt/submit → action coordinator/task, topic=prompt/status)");
}

PromptInput::~PromptInput() { shutdown(); }

void PromptInput::status(const std::string& goal_id, const std::string& text, const std::string& json) {
  // coordinator JSON 앞에 goal_id·text 를 붙인다
  std::string body = json.size() > 2 && json.front() == '{' ? json.substr(1) : std::string("\"raw\":") + json_quoted(json) + "}";
  publish<srf::msgs::String>("prompt/status", srf::msgs::String{"{\"goal_id\":" + json_quoted(goal_id) + ",\"text\":" +
                                                                json_quoted(text) + "," + body});
}

srf::msgs::String PromptInput::submit(const srf::msgs::String& request) {
  const std::string text = trim(request.data);
  if (text.empty()) return srf::msgs::String{R"({"accepted":false,"error":"빈 명령"})"};
  if (is_stop(text)) {
    try {
      (void)context().services().call<srf::msgs::TriggerRequest, srf::msgs::TriggerResponse>(
          "coordinator/stop", srf::msgs::TriggerRequest{});
    } catch (const std::exception& error) {
      return srf::msgs::String{"{\"accepted\":false,\"error\":" + json_quoted(error.what()) + "}"};
    }
    return srf::msgs::String{R"({"accepted":true,"tool":"stop"})"};
  }
  try {
    auto holder = std::make_shared<std::string>();
    auto handle = task_->send_goal(srf::msgs::String{text}, [this, text, holder](const srf::msgs::String& feedback) {
      status(*holder, text, feedback.data);
    });
    *holder = handle->goal_id();
    status(handle->goal_id(), text, R"({"stage":"queued"})");
    // 끝(done/error/hold)도 coordinator 가 feedback 으로 보낸다 → 그대로 prompt/status 로 간다
    return srf::msgs::String{"{\"accepted\":true,\"goal_id\":" + json_quoted(handle->goal_id()) + "}"};
  } catch (const std::exception& error) {
    return srf::msgs::String{"{\"accepted\":false,\"error\":" + json_quoted(std::string("coordinator 없음: ") + error.what()) + "}"};
  }
}

}  // namespace srf_recycling_ws
