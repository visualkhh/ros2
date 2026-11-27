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

#include "krosc/control/controller_manager.hpp"
#include "krosc/msgs.hpp"
#include "krosc/tf/math.hpp"

#include "llm_json.hpp"
#include "command_rules.hpp"

namespace krosc_ws {

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
- vlm_query: ask the camera/scene a question. Put the question in question.
- vla_pick: use the learned VLA policy, only when the user explicitly asks for it (red/blue cube).
- unsupported: multiple actions, missing distance/preset, or anything else.
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

}  // namespace

LlmInput::LlmInput(krosc::Context& context, std::string ollama_url, std::string model,
                   double timeout_s)
    : Node(context, "llm_input"),
      client_(std::make_unique<OllamaClient>(std::move(ollama_url), std::move(model),
                                             timeout_s)),
      move_(std::make_unique<MoveClient>(context, krosc::msgs::moveit::MOVE_ACTION)) {
  odom_subscription_ = subscribe<krosc::msgs::Odometry>(
      "odom", [this](const krosc::msgs::Odometry& message) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_x_ = message.pose.pose.position.x;
        odom_y_ = message.pose.pose.position.y;
        odom_yaw_ = krosc::tfmath::yaw_of(message.pose.pose.orientation);
        odom_seen_ = true;
      });
  drive_timer_ = create_timer(0.1, [this] { drive_tick(); });
  create_service<krosc::msgs::String, krosc::msgs::String>(
      "llm/command", [this](const krosc::msgs::String& request) {
        return handle_command(request);
      });
  worker_ = std::thread([this] { worker_loop(); });
  logger().info("LlmInput: ready (service=llm/command, topic=llm/status)");
}

LlmInput::~LlmInput() {
  stopping_.store(true);
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

void LlmInput::emergency_stop() {
  dance_cancel_.store(true);
  stop_drive();
  for (const char* service : {"vla/stop", "pick/stop", "nav/stop"}) {
    try {
      (void)context().services().call<krosc::msgs::TriggerRequest,
                                      krosc::msgs::TriggerResponse>(
          service, krosc::msgs::TriggerRequest{});
    } catch (const std::exception&) {
    }
  }
  std::lock_guard<std::mutex> lock(move_mutex_);
  if (move_ && !move_goal_id_.empty()) {
    move_->cancel(move_goal_id_);
    move_goal_id_.clear();
  }
}

krosc::msgs::String LlmInput::handle_command(const krosc::msgs::String& request) {
  const std::string text = trim(request.data);
  if (text.empty()) {
    return krosc::msgs::String{R"({"accepted":false,"error":"빈 명령"})"};
  }
  const std::string keyword = lower(text);
  if (keyword == "stop" || text == "정지" || text == "멈춰") {
    emergency_stop();
    return krosc::msgs::String{R"({"accepted":true,"tool":"stop"})"};
  }
  Request queued;
  queued.id = next_id_.fetch_add(1);
  queued.text = text;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back(queued);
  }
  queue_cv_.notify_one();
  return krosc::msgs::String{"{\"accepted\":true,\"request_id\":" +
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
  publish_status(request.id, "thinking");
  llm::Plan plan;
  std::string source = "rule";
  if (const auto rule = llm::parse_command(request.text)) {
    plan = *rule;
  } else {
    source = "llm";
    const OllamaReply reply = client_->chat(kSystemPrompt, request.text, kSchema);
    if (!reply.ok) {
      publish_status(request.id, "error", ",\"error\":" + json_quoted(reply.error));
      return;
    }
    const auto parsed = llm::parse_plan(reply.content);
    if (!parsed) {
      publish_status(request.id, "error",
                     ",\"error\":" + json_quoted("모델 응답 파싱 실패: " + reply.content.substr(0, 160)));
      return;
    }
    plan = *parsed;
    apply_direction_hint(request.text, plan);
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
                 ",\"source\":" + json_quoted(source) + ",\"plan\":" + plan_json);
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
    publish_status(id, "done", ",\"tool\":" + json_quoted(intent) +
                                   ",\"target\":" +
                                   json_quoted(intent == "place" ? place_target : color) +
                                   ",\"result\":" + json_quoted(result));
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
    publish_status(id, "done", ",\"tool\":" + json_quoted(intent) +
                                   ",\"result\":" + json_quoted(result));
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
    publish_status(id, "done", ",\"tool\":\"vla_pick\",\"result\":" + json_quoted(result));
    return;
  }
  publish_status(id, "done", ",\"tool\":\"unsupported\"");
}

void LlmInput::dance(std::uint64_t id, const std::string& style) {
  dance_cancel_.store(false);
  stop_drive();
  const bool use_arm = style == "show" || style == "wave";
  const auto switch_arm = [&](bool manual) {
    krosc::control::SwitchControllerRequest request;
    request.start_controllers =
        manual ? std::vector<std::string>{"arm_command_controller"}
               : std::vector<std::string>{"joint_trajectory_controller"};
    request.stop_controllers =
        manual ? std::vector<std::string>{"joint_trajectory_controller"}
               : std::vector<std::string>{"arm_command_controller"};
    request.strictness = 0;
    try {
      (void)context().services().call<krosc::control::SwitchControllerRequest,
                                      krosc::control::SwitchControllerResponse>(
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
    publish<krosc::msgs::Twist>(
        "cmd_vel_nav",
        krosc::msgs::Twist{{linear, 0.0, 0.0}, {0.0, 0.0, angular}});
    if (use_arm && step % 2 == 0) {
      // 5관절을 5Hz로 흔든다. show는 크게, wave는 손목 위주로.
      const double phase = elapsed * (style == "show" ? 4.0 : 6.0);
      const double amplitude = style == "show" ? 1.0 : 0.7;
      krosc::msgs::JointState command;
      command.header.stamp = now();
      command.name = {"joint1", "joint2", "joint3", "joint4",
                      "gripper_left_joint"};
      command.position = {
          std::clamp(0.6 * amplitude * std::sin(phase), -1.2, 1.2),
          std::clamp(-0.25 + 0.25 * std::sin(phase + 1.0), -1.0, 1.0),
          std::clamp(0.45 + 0.20 * std::sin(phase + 2.0), -0.5, 1.2),
          std::clamp(0.50 + 0.30 * std::sin(phase + 3.0), -0.8, 1.5),
          0.015};
      publish<krosc::msgs::JointState>("arm_joint_command", std::move(command));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ++step;
  }
  publish<krosc::msgs::Twist>(
      "cmd_vel_nav", krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
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
  publish<krosc::msgs::Twist>(
      "cmd_vel_nav",
      krosc::msgs::Twist{{drive_speed_ * drive_sign_, 0.0, 0.0}, {0.0, 0.0, 0.0}});
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
  publish<krosc::msgs::Twist>(
      "cmd_vel_nav",
      krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, turn_angular_}});
}

void LlmInput::stop_drive() {
  if (drive_active_.exchange(false)) {
    publish<krosc::msgs::Twist>("cmd_vel_nav",
                                krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  }
}

void LlmInput::send_arm_preset(std::uint64_t id, const std::string& preset) {
  if (!move_) {
    publish_status(id, "error", ",\"error\":\"MoveIt client 없음\"");
    return;
  }
  // VLA/pick이 arm_command_controller를 켜 둘 수 있다. MoveIt 전에 trajectory로 되돌린다.
  {
    krosc::control::SwitchControllerRequest request;
    request.start_controllers = {"joint_trajectory_controller"};
    request.stop_controllers = {"arm_command_controller"};
    request.strictness = 0;
    try {
      (void)context().services().call<krosc::control::SwitchControllerRequest,
                                      krosc::control::SwitchControllerResponse>(
          "controller_manager/switch_controller", request);
    } catch (const std::exception&) {
    }
  }
  krosc::msgs::moveit::MoveGroupGoal goal;
  goal.preset = preset;
  goal.plan_only = false;
  goal.cartesian = true;
  goal.grip = -1.0;
  try {
    const auto handle = move_->send_goal(
        goal, [this, id](const krosc::msgs::moveit::MoveGroupFeedback& feedback) {
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
    if (result.status == krosc::GoalStatus::Succeeded) {
      publish_status(id, "done",
                     ",\"tool\":\"arm_preset\",\"preset\":" + json_quoted(preset));
      return;
    }
    std::string detail = "MoveIt 실패";
    if (result.result.has_value()) {
      try {
        const auto* move_result =
            std::any_cast<krosc::msgs::moveit::MoveGroupResult>(&result.result);
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
        context().services().call<krosc::msgs::String, krosc::msgs::TriggerResponse>(
            "buzzer/melody", krosc::msgs::String{name});
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

std::string LlmInput::call_optional(const std::string& service,
                                    const std::string& payload, std::string& error) {
  try {
    const auto response = context().services().call<krosc::msgs::String,
                                                    krosc::msgs::String>(
        service, krosc::msgs::String{payload});
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
  publish<krosc::msgs::String>("llm/status", krosc::msgs::String{std::move(json)});
}

}  // namespace krosc_ws
