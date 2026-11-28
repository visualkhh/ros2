#include "llm.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "srf/msgs.hpp"

#include "nodes/common/llm_json.hpp"
#include "command_rules.hpp"

namespace srf_recycling_ws {

namespace {

constexpr const char* kSchema = R"({
  "type": "object",
  "properties": {
    "intent": {"type": "string", "enum": ["drive", "arm_preset", "stop", "buzzer", "pick", "place", "hide", "approach", "dance", "vlm_query", "vla_pick", "sort", "unsupported"]},
    "meters": {"type": "number", "description": "forward/backward distance in meters; 0 otherwise"},
    "degrees": {"type": "number", "description": "turn angle in degrees for left/right; 0 otherwise"},
    "direction": {"type": "string", "enum": ["forward", "backward", "left", "right"]},
    "preset": {"type": "string", "enum": ["home", "stow"]},
    "melody": {"type": "string", "enum": ["mario_death", "success", "alarm", "beep"]},
    "style": {"type": "string", "enum": ["dance", "spin", "wiggle", "bow", "celebrate", "show", "wave"]},
    "question": {"type": "string"},
    "target": {"type": "string"}
  },
  "required": ["intent", "meters"]
})";

constexpr const char* kSystemPrompt = R"(You are the command parser for a TurtleBot3 Waffle Pi with an OpenMANIPULATOR-X arm.
Convert the user's Korean or English request into exactly one intent.
- drive: move or turn the base. direction = forward|backward|left|right. meters is REQUIRED and must be a number ("1m" -> 1.0, "50cm" -> 0.5; 0 for left/right turns).
  For left/right, turn in place and put the angle in degrees (default 90 if the user did not say).
  Examples: "뒤로 0.5m" -> direction backward, meters 0.5; "왼쪽으로 90도" -> direction left, degrees 90, meters 0.
- arm_preset: move the arm to a preset. preset must be "home" or "stow".
- stop: stop the robot immediately.
- buzzer: play a buzzer melody. melody = "mario_death" for Mario death/game over, "success", "alarm", or "beep".
- pick: pick up a puck with the manipulator. Put the color (yellow/red/blue) in target.
- place: put the held puck down. target = floor (default) or body (on the robot's own deck).
  Use body ONLY for "내 몸/몸 위" (on me). "내려놔/바닥/다시 내려놔" must be floor.
- hide: move to a place that is not visible from here (behind a wall/cover). No arguments.
- approach: drive forward close to the visible wall or object ahead using the lidar. No arguments.
- dance: entertainment motion. style = dance (default), spin (turn in place), wiggle (shake side to side), bow (greet), celebrate (success melody + dance), show (roam + arm wave + melody), wave (arm wave in place).
  Examples: "춤춰" -> dance style dance; "빙글빙글 돌아" -> spin; "흔들어" -> wiggle; "인사해" -> bow; "축하해" -> celebrate; "노래부르면서 춤춰" -> show; "손 흔들어" -> wave.
- vlm_query: use ONLY when the user asks what the camera sees, asks to describe/identify the scene, or asks a visual question. Put the question in question.
  Examples: "지금 카메라에 뭐가 보여?" -> vlm_query; "앞에 뭐 있어?" -> vlm_query.
  Do NOT use vlm_query for weather, math, general knowledge, battery, or robot internal status. Those are unsupported.
- vla_pick: use the learned VLA policy when the user asks to pick/place a red or blue cube, asks for a VLA/learned policy, or asks for an AI/learned pick. Put the color/object in target.
- pick: deterministic manipulator pick, normally for the yellow puck or known objects. Red/blue cube or "VLA" requests should use vla_pick.
- sort: the recycling mission. Put recyclables on the work mat into their bins. target = can | pet | paper | all.
  Examples: "캔 넣어" -> sort target can; "페트병 치워" -> sort target pet; "종이 분리수거해" -> sort target paper; "전부 분리수거해" -> sort target all.
- unsupported: multiple actions, missing distance/preset, or anything else.
The user message includes a compact JSON snapshot of the robot state (pose/TF, lidar, camera/depth, arm, wheels, battery). Before choosing, use it to check reachability, obstacles, stale sensors, and whether the manipulator is already holding something. If the requested action is unsafe or impossible from the current state, return unsupported.
When the user asks about the camera/scene, choose vlm_query. When the user asks for a red/blue cube, VLA, or a learned pick, choose vla_pick. Use pick for the deterministic yellow puck/known object.
Always include meters in the JSON. Use 0 when the intent is not drive.
Return only the JSON object. Do not explain.)";

std::string trim(std::string text) {
  const auto not_space = [](unsigned char character) { return std::isspace(character) == 0; };
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
  text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
  return text;
}

std::string lower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return text;
}

std::string number(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return buffer;
}

std::string json_quoted(const std::string& value) {
  return "\"" + llm::escape_json(value) + "\"";
}

std::string base64_encode(const std::vector<std::uint8_t>& data) {
  static constexpr char kTable[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((data.size() + 2U) / 3U * 4U);
  for (std::size_t index = 0; index < data.size(); index += 3U) {
    const unsigned first = data[index];
    const unsigned second = index + 1U < data.size() ? data[index + 1U] : 0U;
    const unsigned third = index + 2U < data.size() ? data[index + 2U] : 0U;
    out.push_back(kTable[first >> 2U]);
    out.push_back(kTable[((first & 0x03U) << 4U) | (second >> 4U)]);
    out.push_back(index + 1U < data.size()
                      ? kTable[((second & 0x0FU) << 2U) | (third >> 6U)]
                      : '=');
    out.push_back(index + 2U < data.size() ? kTable[third & 0x3FU] : '=');
  }
  return out;
}

// LLM이 "오른쪽으로 45도"를 arm_preset으로 오분류하는 경우가 있어,
// 방향/회전 문구는 결정적으로 drive로 보정한다.
void apply_direction_hint(const std::string& text, llm::Plan& plan) {
  const std::string lowered = lower(text);
  if (lowered.find("팔") != std::string::npos ||
      lowered.find("매니") != std::string::npos ||
      lowered.find("arm") != std::string::npos) {
    return;
  }
  const bool left = lowered.find("왼") != std::string::npos ||
                    lowered.find("좌") != std::string::npos ||
                    lowered.find("left") != std::string::npos;
  const bool right = lowered.find("오른") != std::string::npos ||
                     lowered.find("우") != std::string::npos ||
                     lowered.find("right") != std::string::npos;
  const bool turn = lowered.find("돌") != std::string::npos ||
                    lowered.find("회전") != std::string::npos ||
                    lowered.find("turn") != std::string::npos ||
                    lowered.find("rotate") != std::string::npos;
  const bool backward = lowered.find("뒤") != std::string::npos ||
                        lowered.find("후진") != std::string::npos ||
                        lowered.find("back") != std::string::npos;

  double value = -1.0;
  for (std::size_t index = 0; index < text.size(); ++index) {
    if (std::isdigit(static_cast<unsigned char>(text[index])) == 0) {
      continue;
    }
    const char* begin = text.c_str() + index;
    char* end = nullptr;
    value = std::strtod(begin, &end);
    if (end != begin) {
      break;
    }
  }

  if ((left || right) && (turn || value >= 0.0)) {
    plan.intent = "drive";
    plan.direction = left ? "left" : "right";
    plan.meters = 0.0;
    plan.degrees = value >= 0.0 ? value : 90.0;
    plan.preset.clear();
    return;
  }
  if (backward && !left && !right) {
    plan.intent = "drive";
    plan.direction = "backward";
    if (plan.meters <= 0.0 && value >= 0.0) {
      plan.meters = value;
    }
    plan.preset.clear();
  }
}

// gemma3:12b가 intent만 맞고 필수 파라미터를 비워 보내는 경우를 보정한다.
// 예: "팔 접어" -> intent=arm_preset, preset 누락.
void apply_parameter_hints(const std::string& text, llm::Plan& plan,
                           const std::optional<llm::Plan>& rule,
                           std::string& source) {
  const std::string lowered = lower(text);
  const auto has = [&lowered](std::initializer_list<const char*> words) {
    for (const char* word : words) {
      if (lowered.find(word) != std::string::npos) {
        return true;
      }
    }
    return false;
  };
  if (plan.intent == "arm_preset") {
    if (plan.preset != "home" && plan.preset != "stow") {
      if (rule && rule->intent == "arm_preset" &&
          (rule->preset == "home" || rule->preset == "stow")) {
        plan = *rule;
        source = "llm+rule";
      } else if (has({"접", "숨", "fold", "stow", "내려", "집어넣"})) {
        plan.preset = "stow";
      } else if (has({"펴", "홈", "home", "기본", "원위치", "초기", "올려"})) {
        plan.preset = "home";
      }
    }
  } else if (plan.intent == "buzzer") {
    if (plan.melody.empty()) {
      if (has({"마리오", "mario", "죽을", "게임오버"})) {
        plan.melody = "mario_death";
      } else if (has({"성공", "축하", "success"})) {
        plan.melody = "success";
      } else if (has({"경고", "알람", "alarm"})) {
        plan.melody = "alarm";
      } else {
        plan.melody = "beep";
      }
    }
  } else if (plan.intent == "dance") {
    if (plan.style.empty()) {
      if (has({"빙글", "spin"})) plan.style = "spin";
      else if (has({"흔들", "wiggle"})) plan.style = "wiggle";
      else if (has({"인사", "절", "bow"})) plan.style = "bow";
      else if (has({"축하", "celebrate"})) plan.style = "celebrate";
      else if (has({"노래", "show"})) plan.style = "show";
      else if (has({"손", "wave"})) plan.style = "wave";
      else plan.style = "dance";
    }
  } else if (plan.intent == "vlm_query") {
    if (plan.question.empty()) {
      plan.question = text;
    }
  } else if (plan.intent == "vla_pick") {
    if (plan.target.empty()) {
      if (lowered.find("파") != std::string::npos ||
          lowered.find("blue") != std::string::npos) {
        plan.target = "blue cube";
      } else {
        plan.target = "red cube";
      }
    }
  } else if (plan.intent == "place") {
    if (plan.target.empty() &&
        (lowered.find("몸") != std::string::npos ||
         lowered.find("body") != std::string::npos)) {
      plan.target = "body";
    }
  }
}

}  // namespace

Llm::Llm(srf::Context& context, std::string ollama_url, std::string model, double timeout_s, std::string mode,
         std::string camera_topic, bool include_image)
    : Node(context, "llm"),
      client_(std::make_unique<OllamaClient>(std::move(ollama_url), std::move(model), timeout_s)),
      mode_(std::move(mode)),
      camera_topic_(std::move(camera_topic)),
      include_image_(include_image) {
  context_subscription_ = subscribe<srf::msgs::String>(
      "robot_context", [this](const srf::msgs::String& message) {
        std::lock_guard<std::mutex> lock(context_mutex_);
        robot_context_ = message.data;
      }, srf::QOS_DEFAULT);
  if (include_image_) {
    image_subscription_ = subscribe<srf::msgs::Image>(
        camera_topic_, [this](const srf::msgs::Image& message) {
          std::lock_guard<std::mutex> lock(image_mutex_);
          image_ = message;
          image_seen_ = true;
        }, srf::QOS_SENSOR_DATA);
  }
  create_service<srf::msgs::String, srf::msgs::String>(
      "llm/parse", [this](const srf::msgs::String& request) {
        const std::string text = trim(request.data);
        if (text.empty()) {
          return srf::msgs::String{R"({"ok":false,"error":"빈 명령"})"};
        }
        return srf::msgs::String{parse(text)};
      });
  logger().info("Llm: ready (service=llm/parse, mode=%s)", mode_.c_str());
}

Llm::~Llm() { shutdown(); }

std::string Llm::parse(const std::string& text) {
  const auto rule = llm::parse_command(text);
  llm::Plan plan;
  std::string source = "rule";
  std::string via = "rule";

  // stop은 서버 장애와 무관하게 즉시 처리한다. 나머지는 LLM이 우선 라우팅한다.
  if (rule && rule->intent == "stop") {
    plan = *rule;
    source = "rule";
    via = "rule (safety stop)";
  } else if (rule && rule->intent == "arm_preset") {
    // 팔 preset은 LLM이 pick 등으로 오분류하는 경우가 있어 결정적으로 처리한다.
    plan = *rule;
    source = "rule";
    via = "rule (arm preset)";
  } else if (mode_ == "rule" && rule) {
    plan = *rule;
    source = "rule";
    via = "rule (mode=rule)";
  } else {
    std::string robot_context;
    {
      std::lock_guard<std::mutex> lock(context_mutex_);
      robot_context = robot_context_;
    }
    const std::string user_text = robot_context.empty()
        ? text
        : "Robot state JSON: " + robot_context + "\nUser command: " + text;
    std::string image_b64;
    if (include_image_) {
      std::lock_guard<std::mutex> lock(image_mutex_);
      if (image_seen_ &&
          (image_.encoding == "jpeg" || image_.encoding == "jpg" ||
           image_.encoding == "png") &&
          !image_.data.empty()) {
        image_b64 = base64_encode(image_.data);
      }
    }
    const OllamaReply reply = client_->chat(kSystemPrompt, user_text, kSchema,
                                            image_b64);
    if (reply.ok) {
      const auto parsed = llm::parse_plan(reply.content);
      if (parsed) {
        if (parsed->intent == "unsupported" && rule) {
          // LLM이 모르는 명령은 검증된 90개 규칙으로 fallback.
          plan = *rule;
          source = "llm+rule";
          via = "LLM→rule (unsupported)";
        } else {
          plan = *parsed;
          source = "llm";
          via = "LLM";
          apply_direction_hint(text, plan);
          const std::string before = source;
          apply_parameter_hints(text, plan, rule, source);
          if (source != before) {
            via = "LLM+rule (parameter fallback)";
          }
        }
      } else if (rule) {
        plan = *rule;
        source = "rule";
        via = "LLM→rule (parse)";
      } else {
        return "{\"ok\":false,\"error\":" + json_quoted("모델 응답 파싱 실패: " + reply.content.substr(0, 160)) + "}";
      }
    } else if (rule) {
      // LLM 서버가 죽어도 알려진 명령은 결정적으로 계속한다.
      plan = *rule;
      source = "rule";
      std::string reason = reply.error.empty() ? "error" : reply.error;
      if (reason.size() > 80) {
        reason.resize(80);
      }
      via = "LLM→rule (" + reason + ")";
    } else {
      return "{\"ok\":false,\"error\":" + json_quoted(reply.error) + "}";
    }
  }
  std::string plan_json = "{\"intent\":" + json_quoted(plan.intent);
  if (plan.meters != 0.0) {
    plan_json += ",\"meters\":" + number(plan.meters);
  }
  if (plan.degrees != 0.0) {
    plan_json += ",\"degrees\":" + number(plan.degrees);
  }
  if (!plan.direction.empty()) {
    plan_json += ",\"direction\":" + json_quoted(plan.direction);
  }
  if (!plan.preset.empty()) {
    plan_json += ",\"preset\":" + json_quoted(plan.preset);
  }
  if (!plan.melody.empty()) {
    plan_json += ",\"melody\":" + json_quoted(plan.melody);
  }
  if (!plan.question.empty()) {
    plan_json += ",\"question\":" + json_quoted(plan.question);
  }
  if (!plan.target.empty()) {
    plan_json += ",\"target\":" + json_quoted(plan.target);
  }
  if (!plan.style.empty()) {
    plan_json += ",\"style\":" + json_quoted(plan.style);
  }
  plan_json += "}";
  return "{\"ok\":true,\"source\":" + json_quoted(source) + ",\"via\":" + json_quoted(via) +
         ",\"plan\":" + plan_json + "}";
}

}  // namespace srf_recycling_ws
