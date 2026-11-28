#include "command_rules.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <string>
#include <string_view>

namespace srf_default_ws::llm {

namespace {

std::string lower_ascii(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return text;
}

bool has(std::string_view text, std::string_view needle) {
  return text.find(needle) != std::string_view::npos;
}

bool has_any(std::string_view text, std::initializer_list<std::string_view> needles) {
  for (const std::string_view needle : needles) {
    if (has(text, needle)) {
      return true;
    }
  }
  return false;
}

struct Quantity {
  bool found = false;
  double value = 0.0;
  enum class Unit { None, Meter, Centimeter, Millimeter, Degree } unit = Unit::None;
};

Quantity first_quantity(const std::string& text) {
  Quantity quantity;
  for (std::size_t index = 0; index < text.size(); ++index) {
    const unsigned char character = static_cast<unsigned char>(text[index]);
    if (std::isdigit(character) == 0) {
      continue;
    }
    const char* begin = text.c_str() + index;
    char* end = nullptr;
    const double value = std::strtod(begin, &end);
    if (end == begin || !std::isfinite(value)) {
      continue;
    }
    quantity.found = true;
    quantity.value = value;
    std::size_t cursor = static_cast<std::size_t>(end - text.c_str());
    while (cursor < text.size() && text[cursor] == ' ') {
      ++cursor;
    }
    const std::string tail = text.substr(cursor);
    if (tail.rfind("cm", 0) == 0 || tail.rfind("센티", 0) == 0 ||
        tail.rfind("센치", 0) == 0) {
      quantity.unit = Quantity::Unit::Centimeter;
    } else if (tail.rfind("mm", 0) == 0 || tail.rfind("밀리", 0) == 0) {
      quantity.unit = Quantity::Unit::Millimeter;
    } else if (tail.rfind("meter", 0) == 0 || tail.rfind("metre", 0) == 0 ||
               tail.rfind("미터", 0) == 0 || tail.rfind("메터", 0) == 0 ||
               tail.rfind("m", 0) == 0) {
      quantity.unit = Quantity::Unit::Meter;
    } else if (tail.rfind("도", 0) == 0 || tail.rfind("degree", 0) == 0 ||
               tail.rfind("deg", 0) == 0) {
      quantity.unit = Quantity::Unit::Degree;
    }
    return quantity;
  }
  return quantity;
}

double meters_from(const Quantity& quantity, double fallback) {
  if (!quantity.found) {
    return fallback;
  }
  switch (quantity.unit) {
    case Quantity::Unit::Centimeter:
      return quantity.value / 100.0;
    case Quantity::Unit::Millimeter:
      return quantity.value / 1000.0;
    case Quantity::Unit::Meter:
    case Quantity::Unit::None:
      return quantity.value;
    case Quantity::Unit::Degree:
      return fallback;
  }
  return fallback;
}

std::string color_from(std::string_view text) {
  if (has_any(text, {"빨강", "빨간", "레드", "red"})) {
    return "red";
  }
  if (has_any(text, {"파랑", "파란", "블루", "blue"})) {
    return "blue";
  }
  return "yellow";
}

bool has_arm_word(std::string_view text) {
  return has_any(text, {"팔", "arm", "매니", "manipulator", "그리퍼", "gripper"});
}

bool has_place_word(std::string_view text) {
  return has_any(text, {"놓", "내려", "올려", "place", "put", "drop", "down"});
}

}  // namespace

std::optional<Plan> parse_command(std::string_view input) {
  const std::string text = lower_ascii(std::string(input));
  if (text.empty()) {
    return std::nullopt;
  }

  Plan plan;

  // 1) 정지. "stop the arm"도 전체 정지로 본다.
  if (has_any(text, {"멈춰", "정지", "스톱", "그만", "stop", "halt"})) {
    plan.intent = "stop";
    return plan;
  }

  // 2) 팔 프리셋. "팔 숨겨"가 hide로 새지 않도록 먼저 처리한다.
  if (has_arm_word(text)) {
    if (has_any(text, {"스토우", "stow", "접", "숨", "fold", "내려", "집어넣"})) {
      plan.intent = "arm_preset";
      plan.preset = "stow";
      return plan;
    }
    if (has_any(text, {"홈", "home", "기본", "원위치", "초기", "올려", "펴"})) {
      plan.intent = "arm_preset";
      plan.preset = "home";
      return plan;
    }
  }

  // 3) 상판(body) place. "내 몸에 올려놔"가 pick으로 새지 않게 먼저.
  if (has_any(text, {"몸", "상판", "덱", "deck", "body", "on me", "로봇 위"}) &&
      has_place_word(text)) {
    plan.intent = "place";
    plan.target = "body";
    return plan;
  }

  // 4) 바닥 place.
  if (has_any(text, {"바닥", "floor", "내려놔", "내려놓", "put it down", "put down",
                     "놓아", "down"})) {
    plan.intent = "place";
    plan.target = "floor";
    return plan;
  }

  // 5) pick. 색을 같이 뽑는다.
  if (has_any(text, {"집", "잡", "픽", "pick", "grab", "lift", "들어 올려"}) ||
      (has_any(text, {"퍽", "puck", "물건", "공"}) && has(text, "들어"))) {
    plan.intent = "pick";
    plan.target = color_from(text);
    return plan;
  }

  // 6) hide / approach. 지도/라이다 behavior.
  if (has_any(text, {"숨", "안 보이", "가려", "hide", "cover", "behind"})) {
    plan.intent = "hide";
    return plan;
  }
  if (has_any(text, {"벽", "wall", "붙어", "approach", "가까이"})) {
    plan.intent = "approach";
    return plan;
  }

  // 7) 버저. "축하해"(entertainment)와 "축하음"(beep)을 구분한다.
  if (has_any(text, {"마리오", "mario", "게임오버", "game over"}) ||
      (has(text, "죽") && has_any(text, {"소리", "효과음", "음"}))) {
    plan.intent = "buzzer";
    plan.melody = "mario_death";
    return plan;
  }
  if (has_any(text, {"성공", "딩동", "정답", "success", "winner"}) ||
      (has(text, "축하") && has_any(text, {"음", "소리", "sound"}))) {
    plan.intent = "buzzer";
    plan.melody = "success";
    return plan;
  }
  if (has_any(text, {"경고", "알람", "알림", "위험", "alarm", "warning", "삐삐"})) {
    plan.intent = "buzzer";
    plan.melody = "alarm";
    return plan;
  }
  if (has_any(text, {"삐", "비프", "beep", "삑", "톤"})) {
    plan.intent = "buzzer";
    plan.melody = "beep";
    return plan;
  }

  // 8) 엔터테인먼트. "빙글빙글 돌아"가 drive turn으로 새지 않게 여기서.
  // 노래/음악 + 춤/놀기 = 쇼(주행+팔 웨이브+멜로디)
  if (has(text, "쇼") ||
      (has_any(text, {"노래", "음악", "music", "song", "show"}) &&
       has_any(text, {"춤", "dance", "놀", "파티", "party"})) ||
      has_any(text, {"파티", "party", "신나게 놀", "놀아봐"})) {
    plan.intent = "dance";
    plan.style = "show";
    return plan;
  }
  if (has_any(text, {"웨이브", "wave"}) ||
      (has_any(text, {"팔", "손", "arm", "hand"}) && has(text, "흔들"))) {
    plan.intent = "dance";
    plan.style = "wave";
    return plan;
  }
  if (has_any(text, {"축하", "celebrate"})) {
    plan.intent = "dance";
    plan.style = "celebrate";
    return plan;
  }
  if (has_any(text, {"인사", "greet", "고개 숙", "bow"})) {
    plan.intent = "dance";
    plan.style = "bow";
    return plan;
  }
  if (has_any(text, {"빙글", "한 바퀴", "제자리", "spin"})) {
    plan.intent = "dance";
    plan.style = "spin";
    return plan;
  }
  if (has_any(text, {"흔들", "wiggle", "shimmy"})) {
    plan.intent = "dance";
    plan.style = "wiggle";
    return plan;
  }
  if (has_any(text, {"춤", "댄스", "dance", "기뻐"})) {
    plan.intent = "dance";
    plan.style = "dance";
    return plan;
  }

  // 9) 주행. 거리/각도와 방향을 뽑는다.
  const bool left = has_any(text, {"왼", "좌", "left"});
  const bool right = has_any(text, {"오른", "우", "right"});
  const bool backward = has_any(text, {"뒤", "후진", "back", "reverse"});
  const bool forward = has_any(text, {"앞", "전진", "직진", "forward", "go", "move", "advance"});
  const bool turn = has_any(text, {"돌", "회전", "turn", "rotate"});
  const Quantity quantity = first_quantity(text);

  if (left || right) {
    plan.intent = "drive";
    plan.direction = left ? "left" : "right";
    plan.meters = 0.0;
    if (quantity.found && (quantity.unit == Quantity::Unit::Degree ||
                           quantity.unit == Quantity::Unit::None)) {
      plan.degrees = quantity.value;
    } else {
      plan.degrees = 90.0;
    }
    return plan;
  }
  if (backward) {
    plan.intent = "drive";
    plan.direction = "backward";
    plan.meters = meters_from(quantity, 0.5);
    return plan;
  }
  if (forward) {
    plan.intent = "drive";
    plan.direction = "forward";
    plan.meters = meters_from(quantity, 0.5);
    return plan;
  }
  if (turn) {
    plan.intent = "drive";
    plan.direction = "right";
    plan.meters = 0.0;
    plan.degrees = quantity.found ? quantity.value : 90.0;
    return plan;
  }
  if (quantity.found) {
    if (quantity.unit == Quantity::Unit::Degree) {
      plan.intent = "drive";
      plan.direction = "right";
      plan.meters = 0.0;
      plan.degrees = quantity.value;
    } else {
      plan.intent = "drive";
      plan.direction = "forward";
      plan.meters = meters_from(quantity, 0.5);
    }
    return plan;
  }

  return std::nullopt;
}

}  // namespace srf_default_ws::llm
