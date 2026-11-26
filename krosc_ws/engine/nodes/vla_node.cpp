#include "vla_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include "krosc/control/controller_manager.hpp"
#include "krosc/msgs.hpp"

#include "http_client.hpp"
#include "llm_json.hpp"

namespace krosc_ws {

namespace {

constexpr std::array<const char*, 5> kJointNames{
    "joint1", "joint2", "joint3", "joint4", "gripper_left_joint"};

// actions.py LOW/HIGH와 같은 공식 OpenMANIPULATOR-X 범위.
struct JointLimit {
  double lower;
  double upper;
};
constexpr std::array<JointLimit, 5> kJointLimits{{
    {-2.827433388230814, 2.827433388230814},
    {-1.790707812546182, 1.5707963267948966},
    {-0.9424777960769379, 1.382300767579509},
    {-1.790707812546182, 2.0420352248333655},
    {-0.010, 0.019},
}};

std::string number(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.4f", value);
  return buffer;
}

std::string quoted(const std::string& value) {
  return "\"" + llm::escape_json(value) + "\"";
}

double clamp(double value, double lower, double upper) {
  return std::clamp(value, lower, upper);
}

std::string lower_copy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return text;
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

std::vector<double> number_array_field(const std::string& json,
                                       const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  const std::size_t key_position = json.find(needle);
  if (key_position == std::string::npos) {
    return {};
  }
  const std::size_t begin = json.find('[', key_position + needle.size());
  if (begin == std::string::npos) {
    return {};
  }
  std::vector<double> values;
  std::size_t index = begin + 1U;
  while (index < json.size()) {
    while (index < json.size() &&
           (std::isspace(static_cast<unsigned char>(json[index])) != 0 ||
            json[index] == ',')) {
      ++index;
    }
    if (index >= json.size() || json[index] == ']') {
      break;
    }
    const char* start = json.c_str() + index;
    char* end = nullptr;
    const double value = std::strtod(start, &end);
    if (end == start) {
      break;
    }
    values.push_back(value);
    index = static_cast<std::size_t>(end - json.c_str());
  }
  return values;
}

}  // namespace

VlaNode::VlaNode(krosc::Context& context, std::string url, std::string camera_topic,
                 double period_s, double timeout_s)
    : Node(context, "vla"),
      url_(std::move(url)),
      camera_topic_(std::move(camera_topic)),
      period_s_(period_s > 0.0 ? period_s : 0.25),
      timeout_s_(timeout_s > 0.0 ? timeout_s : 180.0) {
  camera_subscription_ = subscribe<krosc::msgs::Image>(
      camera_topic_, [this](const krosc::msgs::Image& message) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        image_ = message;
        image_seen_ = true;
      });
  joint_subscription_ = subscribe<krosc::msgs::JointState>(
      "joint_states", [this](const krosc::msgs::JointState& message) {
        std::array<double, 5> values{};
        std::array<bool, 5> seen{};
        for (std::size_t index = 0; index < message.name.size() &&
                                    index < message.position.size();
             ++index) {
          for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
            if (message.name[index] == kJointNames[joint]) {
              values[joint] = message.position[index];
              seen[joint] = true;
            }
          }
        }
        const bool complete = std::all_of(seen.begin(), seen.end(),
                                          [](bool value) { return value; });
        if (!complete) {
          return;
        }
        std::lock_guard<std::mutex> lock(joints_mutex_);
        joints_ = values;
        joints_seen_ = true;
      });
  odom_subscription_ = subscribe<krosc::msgs::Odometry>(
      "odom", [this](const krosc::msgs::Odometry& message) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        linear_ = message.twist.twist.linear.x;
        yaw_rate_ = message.twist.twist.angular.z;
        odom_seen_ = true;
      });
  timer_ = create_timer(period_s_, [this] { tick(); });
  create_service<krosc::msgs::String, krosc::msgs::String>(
      "vla/command", [this](const krosc::msgs::String& request) {
        return handle_command(request);
      });
  create_service<krosc::msgs::TriggerRequest, krosc::msgs::TriggerResponse>(
      "vla/stop", [this](const krosc::msgs::TriggerRequest& request) {
        return handle_stop(request);
      });
  worker_ = std::thread([this] { worker_loop(); });
  logger().info("VlaNode: ready service=vla/command period=%.2fs sidecar=%s",
                period_s_, url_.c_str());
}

VlaNode::~VlaNode() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    cancel_ = true;
    pending_ = false;
  }
  condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  stop_motion();
  shutdown();
}

krosc::msgs::String VlaNode::handle_command(const krosc::msgs::String& request) {
  std::string target = llm::string_field(request.data, "target").value_or("");
  if (target.empty()) {
    target = request.data.empty() ? "red block" : request.data;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_ || plan_active_) {
    return krosc::msgs::String{
        R"({"accepted":false,"error":"VLA busy","stage":"busy"})"};
  }
  pending_ = true;
  pending_id_ = next_id_++;
  pending_target_ = target;
  cancel_ = false;
  const std::uint64_t id = pending_id_;
  condition_.notify_one();
  return krosc::msgs::String{"{\"accepted\":true,\"request_id\":" +
                             std::to_string(id) + "}"};
}

krosc::msgs::TriggerResponse VlaNode::handle_stop(
    const krosc::msgs::TriggerRequest&) {
  emergency_stop();
  return krosc::msgs::TriggerResponse{true, "VLA stop"};
}

void VlaNode::emergency_stop() {
  std::uint64_t id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_ = true;
    pending_ = false;
    actions_.clear();
    plan_active_ = false;
    id = plan_id_;
  }
  stop_motion();
  if (id != 0) {
    publish_status(id, "cancelled");
  }
  std::string error;
  (void)switch_arm(false, error);
}

void VlaNode::worker_loop() {
  while (true) {
    std::uint64_t id = 0;
    std::string target;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return stopping_ || pending_; });
      if (stopping_) {
        return;
      }
      if (!pending_) {
        continue;
      }
      id = pending_id_;
      target = pending_target_;
      pending_ = false;
    }
    publish_status(id, "planning");
    std::vector<Action> actions;
    std::string instruction;
    std::string error;
    if (!request_plan(id, target, actions, instruction, error)) {
      publish_status(id, "error", ",\"error\":" + quoted(error));
      continue;
    }
    bool cancelled = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cancelled = cancel_;
    }
    if (cancelled) {
      publish_status(id, "cancelled");
      continue;
    }
    std::string switch_error;
    if (!switch_arm(true, switch_error)) {
      publish_status(id, "error",
                     ",\"error\":" + quoted("controller switch: " + switch_error));
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (cancel_) {
        cancelled = true;
      } else {
        actions_.clear();
        for (const Action& action : actions) {
          actions_.push_back(action);
        }
        plan_active_ = true;
        plan_id_ = id;
        plan_index_ = 0;
        plan_total_ = actions.size();
        plan_instruction_ = instruction;
      }
    }
    if (cancelled) {
      publish_status(id, "cancelled");
      std::string ignored;
      (void)switch_arm(false, ignored);
      continue;
    }
    publish_status(id, "executing",
                   ",\"steps\":" + std::to_string(actions.size()) +
                       ",\"instruction\":" + quoted(instruction));
  }
}

void VlaNode::tick() {
  Action action;
  std::uint64_t id = 0;
  std::size_t index = 0;
  std::size_t total = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!plan_active_ || actions_.empty()) {
      return;
    }
    action = actions_.front();
    actions_.pop_front();
    ++plan_index_;
    id = plan_id_;
    index = plan_index_;
    total = plan_total_;
  }
  apply_action(action);
  publish_status(id, "executing",
                 ",\"index\":" + std::to_string(index) +
                     ",\"steps\":" + std::to_string(total));
  if (index >= total) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan_active_ = false;
    }
    stop_motion();
    publish_status(id, "done", ",\"steps\":" + std::to_string(total));
    std::string error;
    (void)switch_arm(false, error);
  }
}

void VlaNode::apply_action(const Action& action) {
  const double linear = clamp(action.value[0], -0.26, 0.26);
  const double angular = clamp(action.value[1], -1.82, 1.82);
  publish<krosc::msgs::Twist>(
      "cmd_vel_nav",
      krosc::msgs::Twist{{linear, 0.0, 0.0}, {0.0, 0.0, angular}});
  krosc::msgs::JointState command;
  command.header.stamp = now();
  command.name.assign(kJointNames.begin(), kJointNames.end());
  command.position.reserve(5);
  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    command.position.push_back(
        clamp(action.value[2 + index], kJointLimits[index].lower,
              kJointLimits[index].upper));
  }
  publish<krosc::msgs::JointState>("arm_joint_command", std::move(command));
}

void VlaNode::stop_motion() {
  publish<krosc::msgs::Twist>(
      "cmd_vel_nav",
      krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
}

void VlaNode::publish_status(std::uint64_t id, const std::string& stage,
                             const std::string& extra_json) {
  std::string json = "{\"request_id\":" + std::to_string(id) + ",\"stage\":" +
                     quoted(stage);
  if (!extra_json.empty()) {
    json += extra_json;
  }
  json += "}";
  publish<krosc::msgs::String>("vla/status",
                               krosc::msgs::String{std::move(json)});
}

bool VlaNode::request_plan(std::uint64_t, const std::string& target,
                           std::vector<Action>& actions, std::string& instruction,
                           std::string& error) {
  krosc::msgs::Image image;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!image_seen_) {
      error = "camera 프레임이 없다";
      return false;
    }
    image = image_;
  }
  std::array<double, 5> joints{};
  {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    if (!joints_seen_) {
      error = "joint_states가 없다";
      return false;
    }
    joints = joints_;
  }
  double linear = 0.0;
  double yaw_rate = 0.0;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!odom_seen_) {
      error = "odom이 없다";
      return false;
    }
    linear = linear_;
    yaw_rate = yaw_rate_;
  }

  const std::string lowered = lower_copy(target);
  if (lowered.find("red") != std::string::npos || target.find("빨") != std::string::npos) {
    instruction = "Pick up the red cube and place it on the green square.";
  } else if (lowered.find("blue") != std::string::npos ||
             target.find("파") != std::string::npos) {
    instruction = "Pick up the blue cube and place it on the green square.";
  } else {
    instruction = target.empty() ? "Pick up the red cube and place it on the green square."
                                 : target;
  }

  std::string state = "[" + number(linear) + "," + number(yaw_rate);
  for (double value : joints) {
    state += "," + number(value);
  }
  state += "]";
  const std::string body =
      "{\"image_b64\":\"" + base64_encode(image.data) + "\",\"encoding\":" +
      quoted(image.encoding) + ",\"width\":" + std::to_string(image.width) +
      ",\"height\":" + std::to_string(image.height) + ",\"instruction\":" +
      quoted(instruction) + ",\"state\":" + state + "}";

  const HttpResponse response = http_post_json(url_, body, timeout_s_);
  if (!response.ok) {
    error = response.error;
    return false;
  }
  const std::vector<double> flat = number_array_field(response.body, "actions_flat");
  if (flat.empty() || flat.size() % 7U != 0U) {
    error = "sidecar 응답에 actions_flat이 없다: " + response.body.substr(0, 160);
    return false;
  }
  actions.clear();
  actions.reserve(flat.size() / 7U);
  for (std::size_t index = 0; index + 7U <= flat.size(); index += 7U) {
    Action action;
    for (std::size_t value = 0; value < 7U; ++value) {
      action.value[value] = flat[index + value];
    }
    actions.push_back(action);
  }
  if (const auto from_reply = llm::string_field(response.body, "instruction")) {
    if (!from_reply->empty()) {
      instruction = *from_reply;
    }
  }
  return true;
}

bool VlaNode::switch_arm(bool manual, std::string& error) {
  krosc::control::SwitchControllerRequest request;
  if (manual) {
    request.start_controllers = {"arm_command_controller"};
    request.stop_controllers = {"joint_trajectory_controller"};
  } else {
    request.start_controllers = {"joint_trajectory_controller"};
    request.stop_controllers = {"arm_command_controller"};
  }
  request.strictness = 0;
  try {
    const auto response = context().services().call<
        krosc::control::SwitchControllerRequest,
        krosc::control::SwitchControllerResponse>(
        "controller_manager/switch_controller", request);
    if (!response.ok) {
      error = response.message;
      return false;
    }
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace krosc_ws
