#pragma once

// LLM 응답 파싱용 최소 JSON. 의존성 없이 Ollama의 {message:{content:"{...}"}}
// 응답과 그 안의 flat plan 객체만 다룬다.
#include <optional>
#include <string>
#include <string_view>

namespace srf_recycling_ws::llm {

struct Plan {
  std::string intent;
  double meters = 0.0;
  double degrees = 0.0;
  std::string direction;
  std::string preset;
  std::string question;
  std::string target;
  std::string melody;
  std::string style;
};

// JSON 문자열 값으로 넣을 수 있게 이스케이프한다. UTF-8은 그대로 통과시킨다.
std::string escape_json(std::string_view text);

// 모델이 ```json ... ```로 감싸 보내는 경우를 벗겨낸다.
std::string strip_code_fence(std::string text);

std::optional<std::string> string_field(std::string_view json, std::string_view key);
std::optional<double> number_field(std::string_view json, std::string_view key);

// Ollama /api/chat 응답 본문에서 message.content를 꺼낸다.
std::optional<std::string> ollama_content(std::string_view response);

// message.content 안의 {"intent":...,"meters":...}를 파싱한다.
std::optional<Plan> parse_plan(std::string_view content);

}  // namespace srf_recycling_ws::llm
