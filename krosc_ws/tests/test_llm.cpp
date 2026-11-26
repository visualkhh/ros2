#include <string>

#include "nodes/llm_json.hpp"

#include "test_framework.hpp"

namespace {

constexpr const char* kOllamaResponse =
    R"({"model":"gemma3:4b","message":{"role":"assistant","content":"{\"intent\":\"drive\",\"meters\":1.25}"},"done":true})";

}  // namespace

KROSC_TEST(llm_extracts_ollama_message_content) {
  const auto content = krosc_ws::llm::ollama_content(kOllamaResponse);
  CHECK(content.has_value());
  CHECK_EQ(std::string(R"({"intent":"drive","meters":1.25})"), *content);
}

KROSC_TEST(llm_parses_drive_plan) {
  const auto plan = krosc_ws::llm::parse_plan(R"({"intent":"drive","meters":0.5})");
  CHECK(plan.has_value());
  CHECK_EQ(std::string("drive"), plan->intent);
  CHECK_NEAR(0.5, plan->meters, 1e-9);
}

KROSC_TEST(llm_parses_arm_preset_plan) {
  const auto plan = krosc_ws::llm::parse_plan(R"({"intent":"arm_preset","preset":"home"})");
  CHECK(plan.has_value());
  CHECK_EQ(std::string("arm_preset"), plan->intent);
  CHECK_EQ(std::string("home"), plan->preset);
}

KROSC_TEST(llm_strips_markdown_fence) {
  const auto plan =
      krosc_ws::llm::parse_plan("```json\n{\"intent\":\"stop\"}\n```");
  CHECK(plan.has_value());
  CHECK_EQ(std::string("stop"), plan->intent);
}

KROSC_TEST(llm_rejects_plan_without_intent) {
  CHECK(!krosc_ws::llm::parse_plan(R"({"meters":1.0})").has_value());
}

KROSC_TEST(llm_escapes_json_strings) {
  const std::string text = "앞으로 \"1m\"\n전진";
  const std::string escaped = krosc_ws::llm::escape_json(text);
  CHECK(escaped.find("\\\"") != std::string::npos);
  CHECK(escaped.find("\\n") != std::string::npos);
  CHECK(escaped.find("앞으로") != std::string::npos);
}

KROSC_TEST(llm_parses_unicode_escape) {
  const auto plan = krosc_ws::llm::parse_plan(
      R"({"intent":"vlm_query","question":"\ubb50 \ubcf4\uc5ec?"})");
  CHECK(plan.has_value());
  CHECK_EQ(std::string("vlm_query"), plan->intent);
  CHECK_EQ(std::string("뭐 보여?"), plan->question);
}
