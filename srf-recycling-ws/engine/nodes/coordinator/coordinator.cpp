#include "coordinator.hpp"

#include <algorithm>
#include <any>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "srf/control/controller_manager.hpp"
#include "srf/nodes/map/map_marker.hpp"
#include "srf/tf/math.hpp"

#include "nodes/common/llm_json.hpp"

namespace srf_recycling_ws {

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string trim(std::string text) {
  const auto not_space = [](unsigned char character) { return std::isspace(character) == 0; };
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
  text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
  return text;
}

std::string lower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
  return text;
}

std::string number(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return buffer;
}

std::string json_quoted(const std::string& value) { return "\"" + llm::escape_json(value) + "\""; }

double wrap_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// JSON 안의 "key":{...} / "key":[...] 를 그대로 잘라낸다 (중첩 괄호·문자열을 센다).
std::string raw_field(const std::string& json, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  auto at = json.find(needle);
  if (at == std::string::npos) return {};
  at = json.find_first_of("{[", at + needle.size());
  if (at == std::string::npos) return {};
  int depth = 0;
  bool in_string = false;
  for (std::size_t i = at; i < json.size(); ++i) {
    const char c = json[i];
    if (in_string) {
      if (c == '\\') ++i;
      else if (c == '"') in_string = false;
      continue;
    }
    if (c == '"') in_string = true;
    else if (c == '{' || c == '[') ++depth;
    else if ((c == '}' || c == ']') && --depth == 0) return json.substr(at, i - at + 1);
  }
  return {};
}

const char* category_word(const std::string& category) {
  if (category == "can") return "can";
  if (category == "pet") return "PET bottle";
  if (category == "paper") return "paper";
  return "item";
}

}  // namespace

Coordinator::Coordinator(srf::Context& context)
    : Node(context, "coordinator"),
      move_(std::make_unique<MoveClient>(context, srf::msgs::moveit::MOVE_ACTION)),
      nav_(std::make_unique<NavClient>(context, srf::msgs::nav2::NAVIGATE_TO_POSE_ACTION)),
      vla_(std::make_unique<StringClient>(context, "vla/execute")) {
  context_subscription_ = subscribe<srf::msgs::String>(
      "robot_context", [this](const srf::msgs::String& message) {
        std::lock_guard<std::mutex> lock(context_mutex_);
        robot_context_ = message.data;
      }, srf::QOS_DEFAULT);
  odom_subscription_ = subscribe<srf::msgs::Odometry>("odom", [this](const srf::msgs::Odometry& message) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_x_ = message.pose.pose.position.x;
    odom_y_ = message.pose.pose.position.y;
    odom_yaw_ = srf::tfmath::yaw_of(message.pose.pose.orientation);
    odom_seen_ = true;
  });
  markers_subscription_ = subscribe<srf::msgs::MapMarkers>(
      "map_markers", [this](const srf::msgs::MapMarkers& message) {
        std::lock_guard<std::mutex> lock(markers_mutex_);
        markers_ = message;
        markers_seen_ = true;
      }, srf::QOS_LATCHED);
  const auto bind_action_status = [this](const std::string& topic, auto& subscription) {
    subscription = subscribe<srf::msgs::String>(
        topic, [this, topic](const srf::msgs::String& message) {
          ActionStatus status;
          status.id = static_cast<std::uint64_t>(llm::number_field(message.data, "request_id").value_or(0.0));
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
  bind_action_status("nav/status", nav_status_subscription_);
  drive_timer_ = create_timer(0.1, [this] { drive_tick(); });
  server_ = std::make_unique<TaskServer>(context, "coordinator/task",
                                         [this](TaskHandle& handle) { return execute(handle); });
  create_service<srf::msgs::TriggerRequest, srf::msgs::TriggerResponse>(
      "coordinator/stop", [this](const srf::msgs::TriggerRequest&) {
        emergency_stop();
        return srf::msgs::TriggerResponse{true, "stopped"};
      });
  logger().info("Coordinator: ready (action=coordinator/task, service=coordinator/stop)");
}

Coordinator::~Coordinator() {
  action_cancel_.store(true);
  action_cv_.notify_all();
  drive_active_.store(false);
  server_.reset();
  shutdown();
}

void Coordinator::emergency_stop() {
  dance_cancel_.store(true);
  action_cancel_.store(true);
  action_cv_.notify_all();
  stop_drive();
  for (const char* service : {"vla/stop", "pick/stop", "nav/stop"}) {
    try {
      (void)context().services().call<srf::msgs::TriggerRequest, srf::msgs::TriggerResponse>(
          service, srf::msgs::TriggerRequest{});
    } catch (const std::exception&) {
    }
  }
  std::lock_guard<std::mutex> lock(move_mutex_);
  if (move_ && !move_goal_id_.empty()) move_->cancel(move_goal_id_);
  if (nav_ && !nav_goal_id_.empty()) nav_->cancel(nav_goal_id_);
  if (vla_ && !vla_goal_id_.empty()) vla_->cancel(vla_goal_id_);
  move_goal_id_.clear();
  nav_goal_id_.clear();
  vla_goal_id_.clear();
}

void Coordinator::report(const std::string& stage, const std::string& extra_json) {
  std::string json = "{\"task_id\":" + std::to_string(task_id_) + ",\"stage\":" + json_quoted(stage);
  if (!extra_json.empty()) json += extra_json;
  json += "}";
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (stage == "done" || stage == "error" || stage == "hold") final_json_ = json;
  if (current_ != nullptr) current_->publish_feedback(srf::msgs::String{json});
}

srf::msgs::String Coordinator::execute(TaskHandle& handle) {
  const std::string text = trim(handle.goal().data);
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    current_ = &handle;
    final_json_.clear();
    task_id_ = next_id_.fetch_add(1);
  }
  action_cancel_.store(false);
  dance_cancel_.store(false);
  {
    std::lock_guard<std::mutex> lock(context_mutex_);
    action_front_baseline_.store(llm::number_field(robot_context_, "front").value_or(0.0));
  }
  // 취소 요청을 하던 동작에도 전한다
  std::atomic<bool> finished{false};
  std::thread watcher([&] {
    while (!finished.load()) {
      if (handle.is_cancel_requested() && !action_cancel_.load()) emergency_stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });
  report("thinking", ",\"text\":" + json_quoted(text));
  std::string error;
  const std::string parsed = call_optional("llm/parse", text, error);
  if (!error.empty() || !llm::bool_field(parsed, "ok").value_or(false)) {
    report("error", ",\"error\":" + json_quoted(error.empty() ? llm::string_field(parsed, "error").value_or("llm 실패")
                                                               : "llm 노드 없음: " + error));
  } else {
    const std::string plan = raw_field(parsed, "plan");
    report("plan", ",\"source\":" + json_quoted(llm::string_field(parsed, "source").value_or("")) +
                       ",\"via\":" + json_quoted(llm::string_field(parsed, "via").value_or("")) +
                       ",\"plan\":" + (plan.empty() ? "{}" : plan));
    dispatch(llm::string_field(plan, "intent").value_or("unsupported"), llm::number_field(plan, "meters").value_or(0.0),
             llm::number_field(plan, "degrees").value_or(0.0), llm::string_field(plan, "direction").value_or(""),
             llm::string_field(plan, "preset").value_or(""), llm::string_field(plan, "melody").value_or(""),
             llm::string_field(plan, "question").value_or(""), llm::string_field(plan, "target").value_or(""),
             llm::string_field(plan, "style").value_or(""));
  }
  finished.store(true);
  watcher.join();
  std::lock_guard<std::mutex> lock(task_mutex_);
  current_ = nullptr;
  if (final_json_.empty()) final_json_ = "{\"task_id\":" + std::to_string(task_id_) + ",\"stage\":\"done\"}";
  return srf::msgs::String{final_json_};
}

void Coordinator::wait_drive() {
  while (drive_active_.load() && !cancelled()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (cancelled()) stop_drive();
}

bool Coordinator::navigate_to_marker(const std::string& marker, std::string& error) {
  srf::msgs::MapMarker target;
  {
    std::lock_guard<std::mutex> lock(markers_mutex_);
    const auto* found = markers_seen_ ? srf::nodes::find_marker(markers_, marker) : nullptr;
    if (found == nullptr) {
      error = "지도 마커가 없다: " + marker;
      return false;
    }
    target = *found;
  }
  srf::msgs::nav2::NavigateToPoseGoal goal;
  goal.pose.header = srf::msgs::Header{now(), target.frame_id.empty() ? "map" : target.frame_id};
  goal.pose.pose = target.pose;
  report("navigating", ",\"marker\":" + json_quoted(marker));
  try {
    const auto handle = nav_->send_goal(goal, [this, marker](const srf::msgs::nav2::NavigateToPoseFeedback& feedback) {
      report("navigating", ",\"marker\":" + json_quoted(marker) + ",\"remaining\":" + number(feedback.distance_remaining));
    });
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      nav_goal_id_ = handle->goal_id();
    }
    const auto result = handle->result(180.0);
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      nav_goal_id_.clear();
    }
    if (result.status == srf::GoalStatus::Succeeded) return true;
    error = cancelled() ? "취소" : "이동 실패: " + marker;
    if (const auto* nav = std::any_cast<srf::msgs::nav2::NavigateToPoseResult>(&result.result);
        nav != nullptr && !nav->error_msg.empty()) {
      error += " (" + nav->error_msg + ")";
    }
  } catch (const std::exception& exception) {
    error = std::string("nav 없음: ") + exception.what();
  }
  return false;
}

std::optional<std::string> Coordinator::vla_execute(const std::string& goal_json, std::string& error) {
  try {
    const auto handle = vla_->send_goal(srf::msgs::String{goal_json}, [this](const srf::msgs::String& feedback) {
      report("vla", ",\"vla\":" + (feedback.data.empty() ? std::string("{}") : feedback.data));
    });
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      vla_goal_id_ = handle->goal_id();
    }
    const auto result = handle->result(120.0);
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      vla_goal_id_.clear();
    }
    const auto* data = std::any_cast<srf::msgs::String>(&result.result);
    const std::string json = data != nullptr ? data->data : std::string{};
    if (result.status == srf::GoalStatus::Succeeded && llm::bool_field(json, "ok").value_or(false)) return json;
    error = cancelled() ? "취소" : llm::string_field(json, "error").value_or("vla 실패");
  } catch (const std::exception& exception) {
    error = std::string("vla 노드 없음: ") + exception.what();
  }
  return std::nullopt;
}

// 분리수거 미션: 대상이 없어질 때까지 [매트로 → 고르기 → 집기 → 바구니로 → 넣기]. 보류는 멈추고 이유를 남긴다.
void Coordinator::sort(const std::string& target) {
  std::map<std::string, int> placed;
  std::string holds = "[";
  const auto summary = [&](const std::string& extra) {
    std::string counts = "{";
    for (const auto& [category, n] : placed) counts += (counts.size() > 1 ? "," : "") + json_quoted(category) + ":" + std::to_string(n);
    return ",\"tool\":\"sort\",\"target\":" + json_quoted(target) + ",\"placed\":" + counts + "},\"holds\":" + holds + "]" + extra;
  };
  constexpr int kMaxItems = 12;  // 무한 반복 방지 (매트 하나에 이보다 많이 놓이지 않는다)
  for (int round = 0; round < kMaxItems; ++round) {
    std::string error;
    if (cancelled()) {
      report("error", summary(",\"error\":\"취소\""));
      return;
    }
    if (!navigate_to_marker("work_mat_approach", error)) {
      report("error", summary(",\"error\":" + json_quoted(error)));
      return;
    }
    report("selecting", ",\"round\":" + std::to_string(round + 1) + ",\"target\":" + json_quoted(target));
    const std::string selected = call_optional("vlm/select", "{\"target\":" + json_quoted(target) + "}", error);
    if (!error.empty()) {
      report("error", summary(",\"error\":" + json_quoted("vlm 노드 없음: " + error)));
      return;
    }
    const std::string decision = llm::string_field(selected, "decision").value_or("hold");
    const std::string reason = llm::string_field(selected, "reason").value_or("unknown");
    if (decision != "select") {
      holds += (holds.size() > 1 ? "," : "") + selected;
      if (reason == "absent") {  // 더 없다 → 끝
        report("done", summary(",\"finished\":\"absent\""));
      } else {                   // 모호·불확실 → 뒤 동작을 막고 사람에게 알린다 (발제 필수)
        report("hold", summary(",\"reason\":" + json_quoted(reason) + ",\"vlm\":" + selected));
      }
      return;
    }
    const std::string item = raw_field(selected, "target");
    const std::string category = llm::string_field(item, "category").value_or("can");
    const std::string target_xyz = raw_field(selected, "target_xyz");
    if (target_xyz.empty()) {
      report("hold", summary(",\"reason\":\"no_depth\",\"vlm\":" + selected));
      return;
    }
    report("picking", ",\"category\":" + json_quoted(category) + ",\"vlm\":" + selected);
    if (!vla_execute("{\"kind\":\"pick\",\"instruction\":" + json_quoted(std::string("pick up the ") + category_word(category)) +
                         ",\"target_xyz\":" + target_xyz + ",\"bin\":" + json_quoted("bin_" + category) + "}",
                     error)) {
      report("error", summary(",\"error\":" + json_quoted("집기 실패: " + error)));
      return;
    }
    if (!navigate_to_marker("bin_" + category + "_approach", error)) {
      report("error", summary(",\"error\":" + json_quoted(error)));
      return;
    }
    report("placing", ",\"category\":" + json_quoted(category));
    if (!vla_execute("{\"kind\":\"place\",\"instruction\":" +
                         json_quoted(std::string("put the ") + category_word(category) + " in the " + category + " bin") +
                         ",\"bin\":" + json_quoted("bin_" + category) + "}",
                     error)) {
      report("error", summary(",\"error\":" + json_quoted("넣기 실패: " + error)));
      return;
    }
    ++placed[category];
    report("placed", summary(""));
  }
  report("done", summary(",\"finished\":\"limit\""));
}

void Coordinator::dispatch(const std::string& intent, double meters,
                        double degrees, const std::string& direction,
                        const std::string& preset, const std::string& melody,
                        const std::string& question, const std::string& target,
                        const std::string& style) {
  if (intent == "drive") {
    const std::string heading = lower(direction.empty() ? "forward" : direction);
    if (heading == "left" || heading == "right") {
      start_turn(heading, degrees > 0.0 ? degrees : 90.0);
      wait_drive();
      return;
    }
    if (heading != "forward" && heading != "backward") {
      report("error", ",\"error\":\"direction은 forward|backward|left|right\"");
      return;
    }
    if (!(meters > 0.0)) {
      report("error", ",\"error\":\"drive에 meters가 없다\"");
      return;
    }
    start_drive(std::clamp(meters, 0.05, 5.0), heading == "forward" ? 1.0 : -1.0);
    wait_drive();
    return;
  }
  if (intent == "arm_preset") {
    const std::string mode = lower(preset);
    if (mode != "home" && mode != "stow") {
      report("error", ",\"error\":\"preset은 home|stow\"");
      return;
    }
    send_arm_preset(mode);
    return;
  }
  if (intent == "stop") {
    emergency_stop();
    report("done", ",\"tool\":\"stop\"");
    return;
  }
  if (intent == "dance") {
    dance(lower(style.empty() ? "dance" : style));
    return;
  }
  if (intent == "buzzer") {
    const std::string name = lower(melody.empty() ? "beep" : melody);
    std::string error;
    (void)play_melody(name, error);
    if (!error.empty()) {
      report("error", ",\"error\":" + json_quoted("버저 실패: " + error));
      return;
    }
    report("done", ",\"tool\":\"buzzer\",\"melody\":" + json_quoted(name));
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
      report("error",
                     ",\"error\":" + json_quoted("pick node 없음: " + error));
      return;
    }
    const auto action_id = static_cast<std::uint64_t>(
        llm::number_field(result, "request_id").value_or(0.0));
    report("executing",
                   ",\"tool\":" + json_quoted(intent) +
                       ",\"target\":" +
                       json_quoted(intent == "place" ? place_target : color) +
                       ",\"action\":" + result);
    const ActionResult outcome =
        wait_action(intent, "pick/status", action_id, 120.0);
    if (outcome.stage == "done") {
      report("done", ",\"tool\":" + json_quoted(intent) +
                                     ",\"target\":" +
                                     json_quoted(intent == "place" ? place_target : color) +
                                     ",\"result\":" + outcome.json);
      return;
    }
    report("error",
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
      report("error",
                     ",\"error\":" + json_quoted("nav behavior 없음: " + error));
      return;
    }
    const auto action_id = static_cast<std::uint64_t>(
        llm::number_field(result, "request_id").value_or(0.0));
    report("executing",
                   ",\"tool\":" + json_quoted(intent) +
                       ",\"action\":" + result);
    const ActionResult outcome =
        wait_action(intent, "nav/status", action_id, 120.0);
    if (outcome.stage == "done") {
      report("done", ",\"tool\":" + json_quoted(intent) +
                                     ",\"result\":" + outcome.json);
      return;
    }
    report("error",
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
      report("error", ",\"error\":" + json_quoted("VLM node 없음: " + error));
      return;
    }
    report("done", ",\"tool\":\"vlm_query\",\"answer\":" + json_quoted(answer));
    return;
  }
  if (intent == "sort") {
    sort(lower(target.empty() ? "all" : target));
    return;
  }
  report("done", ",\"tool\":\"unsupported\"");
}

void Coordinator::dance(const std::string& style) {
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
  report("dancing",
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
    report("error", ",\"error\":\"dance cancelled\"");
    return;
  }
  report("done",
                 ",\"tool\":\"dance\",\"style\":" + json_quoted(style) +
                     ",\"seconds\":" + number(duration));
}

void Coordinator::start_drive(double meters, double sign) {
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
    report("error", ",\"error\":\"odom이 아직 없다\"");
    return;
  }
  stop_drive();
  drive_mode_ = 0;
  drive_sign_ = sign;
  drive_target_ = meters;
  drive_start_x_ = x;
  drive_start_y_ = y;
  drive_started_ = now();
  drive_last_distance_ = 0.0;
  drive_progress_time_ = drive_started_;
  drive_active_.store(true);
  report("executing",
                 ",\"tool\":\"drive\",\"direction\":" +
                     json_quoted(sign >= 0.0 ? "forward" : "backward") +
                     ",\"meters\":" + number(meters));
}

void Coordinator::start_turn(const std::string& direction, double degrees) {
  double yaw = 0.0;
  bool seen = false;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    yaw = odom_yaw_;
    seen = odom_seen_;
  }
  if (!seen) {
    report("error", ",\"error\":\"odom이 아직 없다\"");
    return;
  }
  const double angle = std::clamp(degrees, 5.0, 360.0);
  stop_drive();
  drive_mode_ = 1;
  drive_sign_ = direction == "left" ? 1.0 : -1.0;
  turn_start_yaw_ = yaw;
  turn_target_ = angle * kPi / 180.0;
  turn_angular_ = 0.8 * drive_sign_;
  drive_started_ = now();
  drive_last_distance_ = 0.0;
  drive_progress_time_ = drive_started_;
  drive_active_.store(true);
  report("executing",
                 ",\"tool\":\"turn\",\"direction\":" + json_quoted(direction) +
                     ",\"degrees\":" + number(angle));
}

void Coordinator::drive_tick() {
  if (!drive_active_.load()) {
    return;
  }
  const std::string safety = safety_stop_reason("drive");
  if (!safety.empty()) {
    stop_drive();
    report("error",
                   ",\"error\":" + json_quoted("안전 정지: " + safety));
    return;
  }
  if (now() - drive_started_ > drive_timeout_) {
    stop_drive();
    report("error",
                   ",\"error\":\"drive timeout\"");
    return;
  }
  if (drive_mode_ == 1) {
    drive_tick_turn();
  } else {
    drive_tick_linear();
  }
}

void Coordinator::drive_tick_linear() {
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
    report("done",
                   ",\"tool\":\"drive\",\"distance\":" + number(distance));
    return;
  }
  // 충돌 모니터/벽에 막히면 명령은 나가는데 odom이 안 늘어난다. 2초 정체면 멈춘다.
  if (distance > drive_last_distance_ + 0.01) {
    drive_last_distance_ = distance;
    drive_progress_time_ = now();
  } else if (now() - drive_progress_time_ > 2.0) {
    stop_drive();
    report("error",
                   ",\"error\":\"drive stalled (collision?)\",\"distance\":" +
                       number(distance));
    return;
  }
  publish<srf::msgs::Twist>(
      "cmd_vel_nav",
      srf::msgs::Twist{{drive_speed_ * drive_sign_, 0.0, 0.0}, {0.0, 0.0, 0.0}});
}

void Coordinator::drive_tick_turn() {
  double yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    yaw = odom_yaw_;
  }
  const double turned = std::abs(wrap_angle(yaw - turn_start_yaw_));
  if (turned + 0.02 >= turn_target_) {
    stop_drive();
    report("done",
                   ",\"tool\":\"turn\",\"degrees\":" +
                       number(turned * 180.0 / kPi));
    return;
  }
  if (turned > drive_last_distance_ + 0.02) {
    drive_last_distance_ = turned;
    drive_progress_time_ = now();
  } else if (now() - drive_progress_time_ > 2.0) {
    stop_drive();
    report("error",
                   ",\"error\":\"turn stalled (collision?)\",\"degrees\":" +
                       number(turned * 180.0 / kPi));
    return;
  }
  publish<srf::msgs::Twist>(
      "cmd_vel_nav",
      srf::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, turn_angular_}});
}

void Coordinator::stop_drive() {
  if (drive_active_.exchange(false)) {
    publish<srf::msgs::Twist>("cmd_vel_nav",
                                srf::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  }
}

void Coordinator::send_arm_preset(const std::string& preset) {
  if (!move_) {
    report("error", ",\"error\":\"MoveIt client 없음\"");
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
        goal, [this](const srf::msgs::moveit::MoveGroupFeedback& feedback) {
          report("moveit",
                         ",\"tool\":\"arm_preset\",\"progress\":" + number(feedback.progress) +
                             ",\"state\":" + json_quoted(feedback.state));
        });
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      move_goal_id_ = handle->goal_id();
    }
    report("executing",
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
      report("done",
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
    report("error", ",\"error\":" + json_quoted(detail));
  } catch (const std::exception& error) {
    report("error", ",\"error\":" + json_quoted(error.what()));
  }
}

std::string Coordinator::play_melody(const std::string& name, std::string& error) {
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

std::string Coordinator::safety_stop_reason(const std::string& tool) const {
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

Coordinator::ActionResult Coordinator::wait_action(const std::string& tool,
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
        report("executing",
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

std::string Coordinator::call_optional(const std::string& service,
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

}  // namespace srf_recycling_ws
