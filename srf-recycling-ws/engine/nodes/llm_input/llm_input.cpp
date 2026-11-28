#include "llm_input.hpp"

#include <algorithm>
#include <any>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "srf/control/controller_manager.hpp"
#include "srf/msgs.hpp"
#include "srf/tf/math.hpp"

#include "nodes/common/llm_json.hpp"
#include "command_rules.hpp"

namespace srf_recycling_ws {

namespace {

constexpr const char* kSchema = R"({
  "type": "object",
  "properties": {
    "intent": {"type": "string", "enum": ["drive", "arm_preset", "stop", "buzzer", "pick", "place", "hide", "approach", "dance", "vlm_query", "vla_pick", "unsupported"]},
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

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

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

LlmInput::LlmInput(srf::Context& context, std::string ollama_url, std::string model,
                   double timeout_s, std::string mode, std::string camera_topic,
                   bool include_image)
    : Node(context, "llm_input"),
      client_(std::make_unique<OllamaClient>(std::move(ollama_url), std::move(model),
                                             timeout_s)),
      move_(std::make_unique<MoveClient>(context, srf::msgs::moveit::MOVE_ACTION)),
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
  odom_subscription_ = subscribe<srf::msgs::Odometry>(
      "odom", [this](const srf::msgs::Odometry& message) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_x_ = message.pose.pose.position.x;
        odom_y_ = message.pose.pose.position.y;
        odom_yaw_ = srf::tfmath::yaw_of(message.pose.pose.orientation);
        odom_seen_ = true;
      });
  const auto bind_action_status = [this](const std::string& topic,
                                         auto& subscription) {
    subscription = subscribe<srf::msgs::String>(
        topic, [this, topic](const srf::msgs::String& message) {
          ActionStatus status;
          status.id = static_cast<std::uint64_t>(
              llm::number_field(message.data, "request_id").value_or(0.0));
          status.stage = llm::string_field(message.data, "stage").value_or("");
          status.json = message.data;
          {
            std::lock_guard<std::mutex> lock(action_mutex_);
            auto& slot = action_status_[topic];
            status.sequence = slot.sequence + 1;
            slot = std::move(status);
          }
          action_cv_.notify_all();
        }, srf::QOS_DEFAULT);
  };
  bind_action_status("pick/status", pick_status_subscription_);
  bind_action_status("vla/status", vla_status_subscription_);
  bind_action_status("nav/status", nav_status_subscription_);
  drive_timer_ = create_timer(0.1, [this] { drive_tick(); });
  create_service<srf::msgs::String, srf::msgs::String>(
      "llm/command", [this](const srf::msgs::String& request) {
        return handle_command(request);
      });
  worker_ = std::thread([this] { worker_loop(); });
  logger().info("LlmInput: ready (service=llm/command, topic=llm/status)");
}

LlmInput::~LlmInput() {
  stopping_.store(true);
  cancel_active_action();
  queue_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  drive_active_.store(false);
  {
    std::lock_guard<std::mutex> lock(move_mutex_);
    if (move_ && !move_goal_id_.empty()) {
      move_->cancel(move_goal_id_);
    }
  }
  shutdown();
}

void LlmInput::cancel_active_action() {
  action_cancel_.store(true);
  action_cv_.notify_all();
}

void LlmInput::emergency_stop() {
  dance_cancel_.store(true);
  cancel_active_action();
  stop_drive();
  for (const char* service : {"vla/stop", "pick/stop", "nav/stop"}) {
    try {
      (void)context().services().call<srf::msgs::TriggerRequest,
                                      srf::msgs::TriggerResponse>(
          service, srf::msgs::TriggerRequest{});
    } catch (const std::exception&) {
    }
  }
  std::lock_guard<std::mutex> lock(move_mutex_);
  if (move_ && !move_goal_id_.empty()) {
    move_->cancel(move_goal_id_);
    move_goal_id_.clear();
  }
}

srf::msgs::String LlmInput::handle_command(const srf::msgs::String& request) {
  const std::string text = trim(request.data);
  if (text.empty()) {
    return srf::msgs::String{R"({"accepted":false,"error":"빈 명령"})"};
  }
  const std::string keyword = lower(text);
  if (keyword == "stop" || text == "정지" || text == "멈춰") {
    emergency_stop();
    return srf::msgs::String{R"({"accepted":true,"tool":"stop"})"};
  }
  Request queued;
  queued.id = next_id_.fetch_add(1);
  queued.text = text;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back(queued);
  }
  queue_cv_.notify_one();
  return srf::msgs::String{"{\"accepted\":true,\"request_id\":" +
                             std::to_string(queued.id) + "}"};
}

void LlmInput::worker_loop() {
  while (true) {
    Request request;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return stopping_.load() || !queue_.empty(); });
      if (queue_.empty()) {
        if (stopping_.load()) {
          return;
        }
        continue;
      }
      request = std::move(queue_.front());
      queue_.pop_front();
    }
    process(request);
  }
}

void LlmInput::process(const Request& request) {
  action_cancel_.store(false);
  {
    std::lock_guard<std::mutex> lock(context_mutex_);
    action_front_baseline_.store(
        llm::number_field(robot_context_, "front").value_or(0.0));
  }
  publish_status(request.id, "thinking");
  const auto rule = llm::parse_command(request.text);
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
        ? request.text
        : "Robot state JSON: " + robot_context + "\nUser command: " + request.text;
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
          apply_direction_hint(request.text, plan);
          const std::string before = source;
          apply_parameter_hints(request.text, plan, rule, source);
          if (source != before) {
            via = "LLM+rule (parameter fallback)";
          }
        }
      } else if (rule) {
        plan = *rule;
        source = "rule";
        via = "LLM→rule (parse)";
      } else {
        publish_status(request.id, "error",
                       ",\"error\":" + json_quoted("모델 응답 파싱 실패: " + reply.content.substr(0, 160)));
        return;
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
      publish_status(request.id, "error", ",\"error\":" + json_quoted(reply.error));
      return;
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
  publish_status(request.id, "plan",
                 ",\"source\":" + json_quoted(source) +
                     ",\"via\":" + json_quoted(via) +
                     ",\"plan\":" + plan_json);
  dispatch(request.id, plan.intent, plan.meters, plan.degrees, plan.direction,
           plan.preset, plan.melody, plan.question, plan.target, plan.style);
}

void LlmInput::dispatch(std::uint64_t id, const std::string& intent, double meters,
                        double degrees, const std::string& direction,
                        const std::string& preset, const std::string& melody,
                        const std::string& question, const std::string& target,
                        const std::string& style) {
  if (intent == "drive") {
    const std::string heading = lower(direction.empty() ? "forward" : direction);
    if (heading == "left" || heading == "right") {
      start_turn(id, heading, degrees > 0.0 ? degrees : 90.0);
      return;
    }
    if (heading != "forward" && heading != "backward") {
      publish_status(id, "error", ",\"error\":\"direction은 forward|backward|left|right\"");
      return;
    }
    if (!(meters > 0.0)) {
      publish_status(id, "error", ",\"error\":\"drive에 meters가 없다\"");
      return;
    }
    start_drive(id, std::clamp(meters, 0.05, 5.0), heading == "forward" ? 1.0 : -1.0);
    return;
  }
  if (intent == "arm_preset") {
    const std::string mode = lower(preset);
    if (mode != "home" && mode != "stow") {
      publish_status(id, "error", ",\"error\":\"preset은 home|stow\"");
      return;
    }
    send_arm_preset(id, mode);
    return;
  }
  if (intent == "stop") {
    emergency_stop();
    publish_status(id, "done", ",\"tool\":\"stop\"");
    return;
  }
  if (intent == "dance") {
    dance(id, lower(style.empty() ? "dance" : style));
    return;
  }
  if (intent == "buzzer") {
    const std::string name = lower(melody.empty() ? "beep" : melody);
    std::string error;
    (void)play_melody(name, error);
    if (!error.empty()) {
      publish_status(id, "error", ",\"error\":" + json_quoted("버저 실패: " + error));
      return;
    }
    publish_status(id, "done", ",\"tool\":\"buzzer\",\"melody\":" + json_quoted(name));
    return;
  }
  if (intent == "pick" || intent == "place") {
    const std::string normalized = lower(target);
    const bool body_place =
        normalized == "body" || normalized == "deck" || normalized == "robot" ||
        target.find("몸") != std::string::npos;
    const std::string place_target = body_place ? "body" : "floor";
    const std::string color = intent == "pick"
                                  ? (target.empty() ? "yellow" : target)
                                  : "yellow";
    const std::string payload = "{\"action\":" + json_quoted(intent) +
                                ",\"color\":" + json_quoted(color) +
                                ",\"target\":" +
                                json_quoted(intent == "place" ? place_target : color) + "}";
    std::string error;
    const std::string result = call_optional("pick/command", payload, error);
    if (!error.empty()) {
      publish_status(id, "error",
                     ",\"error\":" + json_quoted("pick node 없음: " + error));
      return;
    }
    const auto action_id = static_cast<std::uint64_t>(
        llm::number_field(result, "request_id").value_or(0.0));
    publish_status(id, "executing",
                   ",\"tool\":" + json_quoted(intent) +
                       ",\"target\":" +
                       json_quoted(intent == "place" ? place_target : color) +
                       ",\"action\":" + result);
    const ActionResult outcome =
        wait_action(id, intent, "pick/status", action_id, 120.0);
    if (outcome.stage == "done") {
      publish_status(id, "done", ",\"tool\":" + json_quoted(intent) +
                                     ",\"target\":" +
                                     json_quoted(intent == "place" ? place_target : color) +
                                     ",\"result\":" + outcome.json);
      return;
    }
    publish_status(id, "error",
                   ",\"tool\":" + json_quoted(intent) +
                       ",\"error\":" +
                       json_quoted(outcome.detail.empty() ? outcome.stage : outcome.detail) +
                       ",\"result\":" + outcome.json);
    return;
  }
  if (intent == "hide" || intent == "approach") {
    const std::string service = intent == "hide" ? "nav/hide" : "nav/approach";
    std::string error;
    const std::string result = call_optional(service, "{}", error);
    if (!error.empty()) {
      publish_status(id, "error",
                     ",\"error\":" + json_quoted("nav behavior 없음: " + error));
      return;
    }
    const auto action_id = static_cast<std::uint64_t>(
        llm::number_field(result, "request_id").value_or(0.0));
    publish_status(id, "executing",
                   ",\"tool\":" + json_quoted(intent) +
                       ",\"action\":" + result);
    const ActionResult outcome =
        wait_action(id, intent, "nav/status", action_id, 120.0);
    if (outcome.stage == "done") {
      publish_status(id, "done", ",\"tool\":" + json_quoted(intent) +
                                     ",\"result\":" + outcome.json);
      return;
    }
    publish_status(id, "error",
                   ",\"tool\":" + json_quoted(intent) +
                       ",\"error\":" +
                       json_quoted(outcome.detail.empty() ? outcome.stage : outcome.detail) +
                       ",\"result\":" + outcome.json);
    return;
  }
  if (intent == "vlm_query") {
    std::string error;
    const std::string payload = "{\"question\":" + json_quoted(question) + "}";
    const std::string answer = call_optional("vlm/query", payload, error);
    if (!error.empty()) {
      publish_status(id, "error", ",\"error\":" + json_quoted("VLM node 없음: " + error));
      return;
    }
    publish_status(id, "done", ",\"tool\":\"vlm_query\",\"answer\":" + json_quoted(answer));
    return;
  }
  if (intent == "vla_pick") {
    std::string error;
    const std::string payload = "{\"target\":" + json_quoted(target) + "}";
    const std::string result = call_optional("vla/command", payload, error);
    if (!error.empty()) {
      publish_status(id, "error", ",\"error\":" + json_quoted("VLA node 없음: " + error));
      return;
    }
    const auto action_id = static_cast<std::uint64_t>(
        llm::number_field(result, "request_id").value_or(0.0));
    publish_status(id, "executing",
                   ",\"tool\":\"vla_pick\",\"action\":" + result);
    const ActionResult outcome =
        wait_action(id, "vla", "vla/status", action_id, 180.0);
    if (outcome.stage == "done") {
      publish_status(id, "done",
                     ",\"tool\":\"vla_pick\",\"result\":" + outcome.json);
      return;
    }
    publish_status(id, "error",
                   ",\"tool\":\"vla_pick\",\"error\":" +
                       json_quoted(outcome.detail.empty() ? outcome.stage : outcome.detail) +
                       ",\"result\":" + outcome.json);
    return;
  }
  publish_status(id, "done", ",\"tool\":\"unsupported\"");
}

void LlmInput::dance(std::uint64_t id, const std::string& style) {
  dance_cancel_.store(false);
  stop_drive();
  const bool use_arm = style == "show" || style == "wave";
  const auto switch_arm = [&](bool manual) {
    srf::control::SwitchControllerRequest request;
    request.start_controllers =
        manual ? std::vector<std::string>{"arm_command_controller"}
               : std::vector<std::string>{"joint_trajectory_controller"};
    request.stop_controllers =
        manual ? std::vector<std::string>{"joint_trajectory_controller"}
               : std::vector<std::string>{"arm_command_controller"};
    request.strictness = 0;
    try {
      (void)context().services().call<srf::control::SwitchControllerRequest,
                                      srf::control::SwitchControllerResponse>(
          "controller_manager/switch_controller", request);
    } catch (const std::exception&) {
    }
  };
  if (use_arm) {
    switch_arm(true);
  }
  if (style == "celebrate") {
    std::string error;
    (void)play_melody("success", error);
  } else if (style == "show") {
    std::string error;
    (void)play_melody("party", error);
  }
  const auto started = std::chrono::steady_clock::now();
  double duration = 4.0;
  if (style == "spin" || style == "wiggle" || style == "celebrate") {
    duration = 3.0;
  } else if (style == "bow") {
    duration = 2.0;
  } else if (style == "show") {
    duration = 8.0;
  } else if (style == "wave") {
    duration = 3.0;
  }
  publish_status(id, "dancing",
                 ",\"tool\":\"dance\",\"style\":" + json_quoted(style));
  int step = 0;
  while (!dance_cancel_.load()) {
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    if (elapsed >= duration) {
      break;
    }
    double linear = 0.0;
    double angular = 0.0;
    if (style == "spin") {
      angular = 0.9;
    } else if (style == "wiggle") {
      angular = (step % 8 < 4) ? 0.8 : -0.8;
    } else if (style == "bow") {
      linear = (step % 8 < 4) ? 0.07 : -0.07;
    } else if (style == "show") {
      // 돌아다니면서 추기: 앞뒤 + 좌우 회전을 천천히 섞는다.
      linear = 0.08 * std::sin(elapsed * 0.8);
      angular = 0.6 * std::sin(elapsed * 1.3);
    } else if (style == "wave") {
      // 제자리에서 팔만 흔든다.
      linear = 0.0;
      angular = 0.0;
    } else {
      linear = 0.07 * std::sin(elapsed * 3.0);
      angular = 0.7 * std::sin(elapsed * 6.0);
    }
    publish<srf::msgs::Twist>(
        "cmd_vel_nav",
        srf::msgs::Twist{{linear, 0.0, 0.0}, {0.0, 0.0, angular}});
    if (use_arm && step % 2 == 0) {
      // 5관절을 5Hz로 흔든다. show는 크게, wave는 손목 위주로.
      const double phase = elapsed * (style == "show" ? 4.0 : 6.0);
      const double amplitude = style == "show" ? 1.0 : 0.7;
      srf::msgs::JointState command;
      command.header.stamp = now();
      command.name = {"joint1", "joint2", "joint3", "joint4",
                      "gripper_left_joint"};
      command.position = {
          std::clamp(0.6 * amplitude * std::sin(phase), -1.2, 1.2),
          std::clamp(-0.25 + 0.25 * std::sin(phase + 1.0), -1.0, 1.0),
          std::clamp(0.45 + 0.20 * std::sin(phase + 2.0), -0.5, 1.2),
          std::clamp(0.50 + 0.30 * std::sin(phase + 3.0), -0.8, 1.5),
          0.015};
      publish<srf::msgs::JointState>("arm_joint_command", std::move(command));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ++step;
  }
  publish<srf::msgs::Twist>(
      "cmd_vel_nav", srf::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  if (use_arm) {
    switch_arm(false);
  }
  if (dance_cancel_.load()) {
    publish_status(id, "error", ",\"error\":\"dance cancelled\"");
    return;
  }
  publish_status(id, "done",
                 ",\"tool\":\"dance\",\"style\":" + json_quoted(style) +
                     ",\"seconds\":" + number(duration));
}

void LlmInput::start_drive(std::uint64_t id, double meters, double sign) {
  double x = 0.0;
  double y = 0.0;
  bool seen = false;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    x = odom_x_;
    y = odom_y_;
    seen = odom_seen_;
  }
  if (!seen) {
    publish_status(id, "error", ",\"error\":\"odom이 아직 없다\"");
    return;
  }
  stop_drive();
  drive_mode_ = 0;
  drive_request_ = id;
  drive_sign_ = sign;
  drive_target_ = meters;
  drive_start_x_ = x;
  drive_start_y_ = y;
  drive_started_ = now();
  drive_last_distance_ = 0.0;
  drive_progress_time_ = drive_started_;
  drive_active_.store(true);
  publish_status(id, "executing",
                 ",\"tool\":\"drive\",\"direction\":" +
                     json_quoted(sign >= 0.0 ? "forward" : "backward") +
                     ",\"meters\":" + number(meters));
}

void LlmInput::start_turn(std::uint64_t id, const std::string& direction,
                          double degrees) {
  double yaw = 0.0;
  bool seen = false;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    yaw = odom_yaw_;
    seen = odom_seen_;
  }
  if (!seen) {
    publish_status(id, "error", ",\"error\":\"odom이 아직 없다\"");
    return;
  }
  const double angle = std::clamp(degrees, 5.0, 360.0);
  stop_drive();
  drive_mode_ = 1;
  drive_request_ = id;
  drive_sign_ = direction == "left" ? 1.0 : -1.0;
  turn_start_yaw_ = yaw;
  turn_target_ = angle * kPi / 180.0;
  turn_angular_ = 0.8 * drive_sign_;
  drive_started_ = now();
  drive_last_distance_ = 0.0;
  drive_progress_time_ = drive_started_;
  drive_active_.store(true);
  publish_status(id, "executing",
                 ",\"tool\":\"turn\",\"direction\":" + json_quoted(direction) +
                     ",\"degrees\":" + number(angle));
}

void LlmInput::drive_tick() {
  if (!drive_active_.load()) {
    return;
  }
  const std::string safety = safety_stop_reason("drive");
  if (!safety.empty()) {
    stop_drive();
    publish_status(drive_request_, "error",
                   ",\"error\":" + json_quoted("안전 정지: " + safety));
    return;
  }
  if (now() - drive_started_ > drive_timeout_) {
    stop_drive();
    publish_status(drive_request_, "error",
                   ",\"error\":\"drive timeout\"");
    return;
  }
  if (drive_mode_ == 1) {
    drive_tick_turn();
  } else {
    drive_tick_linear();
  }
}

void LlmInput::drive_tick_linear() {
  double x = 0.0;
  double y = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    x = odom_x_;
    y = odom_y_;
  }
  const double distance = std::hypot(x - drive_start_x_, y - drive_start_y_);
  if (distance >= drive_target_) {
    stop_drive();
    publish_status(drive_request_, "done",
                   ",\"tool\":\"drive\",\"distance\":" + number(distance));
    return;
  }
  // 충돌 모니터/벽에 막히면 명령은 나가는데 odom이 안 늘어난다. 2초 정체면 멈춘다.
  if (distance > drive_last_distance_ + 0.01) {
    drive_last_distance_ = distance;
    drive_progress_time_ = now();
  } else if (now() - drive_progress_time_ > 2.0) {
    stop_drive();
    publish_status(drive_request_, "error",
                   ",\"error\":\"drive stalled (collision?)\",\"distance\":" +
                       number(distance));
    return;
  }
  publish<srf::msgs::Twist>(
      "cmd_vel_nav",
      srf::msgs::Twist{{drive_speed_ * drive_sign_, 0.0, 0.0}, {0.0, 0.0, 0.0}});
}

void LlmInput::drive_tick_turn() {
  double yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    yaw = odom_yaw_;
  }
  const double turned = std::abs(wrap_angle(yaw - turn_start_yaw_));
  if (turned + 0.02 >= turn_target_) {
    stop_drive();
    publish_status(drive_request_, "done",
                   ",\"tool\":\"turn\",\"degrees\":" +
                       number(turned * 180.0 / kPi));
    return;
  }
  if (turned > drive_last_distance_ + 0.02) {
    drive_last_distance_ = turned;
    drive_progress_time_ = now();
  } else if (now() - drive_progress_time_ > 2.0) {
    stop_drive();
    publish_status(drive_request_, "error",
                   ",\"error\":\"turn stalled (collision?)\",\"degrees\":" +
                       number(turned * 180.0 / kPi));
    return;
  }
  publish<srf::msgs::Twist>(
      "cmd_vel_nav",
      srf::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, turn_angular_}});
}

void LlmInput::stop_drive() {
  if (drive_active_.exchange(false)) {
    publish<srf::msgs::Twist>("cmd_vel_nav",
                                srf::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  }
}

void LlmInput::send_arm_preset(std::uint64_t id, const std::string& preset) {
  if (!move_) {
    publish_status(id, "error", ",\"error\":\"MoveIt client 없음\"");
    return;
  }
  // VLA/pick이 arm_command_controller를 켜 둘 수 있다. MoveIt 전에 trajectory로 되돌린다.
  {
    srf::control::SwitchControllerRequest request;
    request.start_controllers = {"joint_trajectory_controller"};
    request.stop_controllers = {"arm_command_controller"};
    request.strictness = 0;
    try {
      (void)context().services().call<srf::control::SwitchControllerRequest,
                                      srf::control::SwitchControllerResponse>(
          "controller_manager/switch_controller", request);
    } catch (const std::exception&) {
    }
  }
  srf::msgs::moveit::MoveGroupGoal goal;
  goal.preset = preset;
  goal.plan_only = false;
  goal.cartesian = true;
  goal.grip = -1.0;
  try {
    const auto handle = move_->send_goal(
        goal, [this, id](const srf::msgs::moveit::MoveGroupFeedback& feedback) {
          publish_status(id, "moveit",
                         ",\"tool\":\"arm_preset\",\"progress\":" + number(feedback.progress) +
                             ",\"state\":" + json_quoted(feedback.state));
        });
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      move_goal_id_ = handle->goal_id();
    }
    publish_status(id, "executing",
                   ",\"tool\":\"arm_preset\",\"preset\":" + json_quoted(preset));
    // MoveGroup 결과가 올 때까지 워커에서 기다린다. 도중에 정지 요청이 오면
    // handle_command의 fast-path가 goal을 취소하고, 여기서는 결과만 정리한다.
    const auto result = handle->result(60.0);
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      if (move_goal_id_ == handle->goal_id()) {
        move_goal_id_.clear();
      }
    }
    if (result.status == srf::GoalStatus::Succeeded) {
      publish_status(id, "done",
                     ",\"tool\":\"arm_preset\",\"preset\":" + json_quoted(preset));
      return;
    }
    std::string detail = "MoveIt 실패";
    if (result.result.has_value()) {
      try {
        const auto* move_result =
            std::any_cast<srf::msgs::moveit::MoveGroupResult>(&result.result);
        if (move_result != nullptr && !move_result->error_msg.empty()) {
          detail = move_result->error_msg;
        }
      } catch (const std::bad_any_cast&) {
      }
    }
    publish_status(id, "error", ",\"error\":" + json_quoted(detail));
  } catch (const std::exception& error) {
    publish_status(id, "error", ",\"error\":" + json_quoted(error.what()));
  }
}

std::string LlmInput::play_melody(const std::string& name, std::string& error) {
  try {
    const auto response =
        context().services().call<srf::msgs::String, srf::msgs::TriggerResponse>(
            "buzzer/melody", srf::msgs::String{name});
    if (!response.success) {
      error = response.message;
      return {};
    }
    return response.message;
  } catch (const std::exception& exception) {
    error = exception.what();
    return {};
  }
}

std::string LlmInput::safety_stop_reason(const std::string& tool) const {
  std::string context;
  {
    std::lock_guard<std::mutex> lock(context_mutex_);
    context = robot_context_;
  }
  if (context.empty()) {
    return {};
  }
  const auto battery = llm::number_field(context, "percent");
  if (battery && *battery < 10.0) {
    return "배터리 부족 (" + number(*battery) + "%)";
  }
  const auto arm_temperature = llm::number_field(context, "max_temperature");
  if (arm_temperature && *arm_temperature >= 75.0) {
    return "매니퓰레이터 과열 (" + number(*arm_temperature) + "°C)";
  }
  const auto front = llm::number_field(context, "front");
  const bool moving = tool == "drive" || tool == "nav" || tool == "vla";
  // 팔이 라이다 전방을 가리는 경우가 있어, 시작 시점보다 새로 가까워진 경우만
  // 장애물로 판단한다. (정지 상태에서 0.05m로 보이는 것은 팔 occlusion)
  const double baseline = action_front_baseline_.load();
  if (moving && front && *front > 0.0 && baseline > 0.0 &&
      *front < 0.12 && *front < baseline - 0.04) {
    return "전방 장애물 (" + number(*front) + " m)";
  }
  return {};
}

LlmInput::ActionResult LlmInput::wait_action(std::uint64_t command_id,
                                             const std::string& tool,
                                             const std::string& topic,
                                             std::uint64_t action_id,
                                             double timeout_s) {
  ActionResult result;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(
                            static_cast<long>(std::max(1.0, timeout_s) * 1000.0));
  std::uint64_t last_sequence = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    if (action_cancel_.load()) {
      result.stage = "cancelled";
      return result;
    }
    const std::string safety = safety_stop_reason(tool);
    if (!safety.empty()) {
      emergency_stop();
      result.stage = "safety";
      result.detail = safety;
      return result;
    }
    std::string stage;
    std::string json;
    std::uint64_t status_id = 0;
    std::uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      const auto found = action_status_.find(topic);
      if (found != action_status_.end()) {
        stage = found->second.stage;
        json = found->second.json;
        status_id = found->second.id;
        sequence = found->second.sequence;
      }
    }
    if (sequence > last_sequence) {
      last_sequence = sequence;
      if (action_id == 0 || status_id == action_id) {
        result.stage = stage;
        result.json = json;
        if (stage == "done" || stage == "error" || stage == "cancelled") {
          result.detail = llm::string_field(json, "error").value_or("");
          return result;
        }
        publish_status(command_id, "executing",
                       ",\"tool\":" + json_quoted(tool) +
                           ",\"action_stage\":" + json_quoted(stage) +
                           ",\"action\":" + json);
      }
    }
    std::unique_lock<std::mutex> lock(action_mutex_);
    action_cv_.wait_for(lock, std::chrono::milliseconds(100));
  }
  emergency_stop();
  result.stage = "timeout";
  result.detail = "동작 완료 대기 시간 초과";
  return result;
}

std::string LlmInput::call_optional(const std::string& service,
                                    const std::string& payload, std::string& error) {
  try {
    const auto response = context().services().call<srf::msgs::String,
                                                    srf::msgs::String>(
        service, srf::msgs::String{payload});
    return response.data;
  } catch (const std::exception& exception) {
    error = exception.what();
    return {};
  }
}

void LlmInput::publish_status(std::uint64_t id, const std::string& stage,
                              const std::string& extra_json) {
  std::string json = "{\"request_id\":" + std::to_string(id) + ",\"stage\":" +
                     json_quoted(stage);
  if (!extra_json.empty()) {
    json += extra_json;
  }
  json += "}";
  publish<srf::msgs::String>("llm/status", srf::msgs::String{std::move(json)});
}

}  // namespace srf_recycling_ws
