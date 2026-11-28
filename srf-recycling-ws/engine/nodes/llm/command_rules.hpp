#pragma once

#include <optional>
#include <string_view>

#include "nodes/common/llm_json.hpp"

namespace srf_recycling_ws::llm {

// 자주 쓰는 한국어/영어 명령을 Ollama 없이 결정적으로 해석한다.
// 매칭되는 규칙이 없으면 nullopt를 돌려주고, 호출자는 LLM으로 폴백한다.
std::optional<Plan> parse_command(std::string_view text);

}  // namespace srf_recycling_ws::llm
