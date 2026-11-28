#include "vla_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <utility>

#include "srf/control/controller_manager.hpp"
#include "srf/nodes/map/map_marker.hpp"
#include "srf/tf/math.hpp"
#include "srf/urdf.hpp"

#include "nodes/common/http_client.hpp"
#include "nodes/common/llm_json.hpp"

namespace srf_recycling_ws {

namespace {

constexpr std::array<const char*, 5> kJointNames{"joint1", "joint2", "joint3", "joint4", "gripper_left_joint"};
// 공식 OpenMANIPULATOR-X 범위
constexpr std::array<std::array<double, 2>, 5> kJointLimits{{{-2.827433388230814, 2.827433388230814},
                                                            {-1.790707812546182, 1.5707963267948966},
                                                            {-0.9424777960769379, 1.382300767579509},
                                                            {-1.790707812546182, 2.0420352248333655},
                                                            {-0.010, 0.019}}};
constexpr double kMaxV = 0.10;  // VLA 접근 속도 상한 (m/s) — Nav 보다 훨씬 느리게
constexpr double kMaxW = 0.50;  // rad/s
constexpr double kBaseLinkZ = 0.010;

std::string json_quoted(const std::string& value) { return "\"" + llm::escape_json(value) + "\""; }

std::string base64_encode(const std::vector<std::uint8_t>& data) {
  static constexpr char kTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((data.size() + 2U) / 3U * 4U);
  for (std::size_t i = 0; i < data.size(); i += 3U) {
    const unsigned a = data[i], b = i + 1U < data.size() ? data[i + 1U] : 0U, c = i + 2U < data.size() ? data[i + 2U] : 0U;
    out.push_back(kTable[a >> 2U]);
    out.push_back(kTable[((a & 0x03U) << 4U) | (b >> 4U)]);
    out.push_back(i + 1U < data.size() ? kTable[((b & 0x0FU) << 2U) | (c >> 6U)] : '=');
    out.push_back(i + 2U < data.size() ? kTable[c & 0x3FU] : '=');
  }
  return out;
}

std::string array_json(const double* v, std::size_t n) {
  std::string out = "[";
  char buffer[32];
  for (std::size_t i = 0; i < n; ++i) {
    std::snprintf(buffer, sizeof(buffer), "%s%.5f", i ? "," : "", std::isfinite(v[i]) ? v[i] : 0.0);
    out += buffer;
  }
  return out + "]";
}

// "key":[x,y,z] 숫자 배열
std::optional<std::array<double, 3>> vec3_field(const std::string& json, const std::string& key) {
  const auto at = json.find("\"" + key + "\"");
  if (at == std::string::npos) return std::nullopt;
  const auto open = json.find('[', at);
  if (open == std::string::npos) return std::nullopt;
  std::array<double, 3> out{};
  const char* p = json.c_str() + open + 1;
  for (auto& value : out) {
    char* end = nullptr;
    value = std::strtod(p, &end);
    if (end == p) return std::nullopt;
    p = end;
    while (*p == ',' || *p == ' ') ++p;
  }
  return out;
}

}  // namespace

VlaNode::VlaNode(srf::Context& context, Options options) : Node(context, "vla"), options_(std::move(options)) {
  if (!options_.urdf.empty()) {
    std::string error;
    chain_ = srf::manipulation::ArmChain::from_urdf(srf::parse_urdf(options_.urdf), "base_link", "end_effector_link",
                                                    {"joint1", "joint2", "joint3", "joint4"}, error);
    if (!chain_) logger().warn("VlaNode: 팔 체인 실패 (엔드이펙터 자세 0): %s", error.c_str());
  }
  tf_listener_ = std::make_unique<srf::TransformListener>(tf_buffer_, *this);
  color_subscription_ = subscribe<srf::msgs::Image>(options_.color_topic, [this](const srf::msgs::Image& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    color_ = m;
    color_seen_ = true;
  }, srf::QOS_SENSOR_DATA);
  depth_subscription_ = subscribe<srf::msgs::Image>(options_.depth_topic, [this](const srf::msgs::Image& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    depth_ = m;
    depth_seen_ = true;
  }, srf::QOS_SENSOR_DATA);
  joints_subscription_ = subscribe<srf::msgs::JointState>("joint_states", [this](const srf::msgs::JointState& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    joints_ = m;
    joints_seen_ = true;
  });
  odom_subscription_ = subscribe<srf::msgs::Odometry>("odom", [this](const srf::msgs::Odometry& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_v_ = m.twist.twist.linear.x;
    odom_w_ = m.twist.twist.angular.z;
    odom_seen_ = true;
  });
  scan_subscription_ = subscribe<srf::msgs::LaserScan>("scan", [this](const srf::msgs::LaserScan& m) {
    // 앞 ±20° 최소 거리 (ml/vla 와 같다)
    double best = 3.5;
    for (std::size_t i = 0; i < m.ranges.size(); ++i) {
      double angle = m.angle_min + m.angle_increment * static_cast<double>(i);
      angle = std::atan2(std::sin(angle), std::cos(angle));
      const double r = m.ranges[i];
      if (std::abs(angle) <= 20.0 * 3.14159265358979323846 / 180.0 && std::isfinite(r) && r > 0.0) best = std::min(best, r);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    front_ = best;
  }, srf::QOS_SENSOR_DATA);
  markers_subscription_ = subscribe<srf::msgs::MapMarkers>("map_markers", [this](const srf::msgs::MapMarkers& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    markers_ = m;
  }, srf::QOS_LATCHED);
  server_ = std::make_unique<Server>(context, "vla/execute", [this](Handle& handle) { return execute(handle); });
  create_service<srf::msgs::TriggerRequest, srf::msgs::TriggerResponse>(
      "vla/stop", [this](const srf::msgs::TriggerRequest&) {
        emergency_stop();
        return srf::msgs::TriggerResponse{true, "vla stop"};
      });
  logger().info("VlaNode: ready action=vla/execute sidecar=%s period=%.2fs replan=%zu", options_.url.c_str(),
                options_.period_s, options_.replan_steps);
}

VlaNode::~VlaNode() {
  stop_.store(true);
  server_.reset();
  shutdown();
}

void VlaNode::emergency_stop() {
  stop_.store(true);
  stop_motion();
}

void VlaNode::stop_motion() { publish<srf::msgs::Twist>("cmd_vel_nav", srf::msgs::Twist{}); }

std::optional<std::string> VlaNode::state_json(const std::optional<srf::msgs::Point>& target_odom,
                                               const std::string& bin, std::string& error) {
  srf::msgs::JointState joints;
  double v = 0.0, w = 0.0, front = 3.5;
  srf::msgs::MapMarkers markers;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!joints_seen_ || !odom_seen_) {
      error = !joints_seen_ ? "joint_states 가 없다" : "odom 이 없다";
      return std::nullopt;
    }
    joints = joints_;
    v = odom_v_;
    w = odom_w_;
    front = front_;
    markers = markers_;
  }
  std::array<double, 5> q{}, dq{}, cur{};
  for (std::size_t j = 0; j < 5; ++j) {
    for (std::size_t i = 0; i < joints.name.size(); ++i) {
      if (joints.name[i] != kJointNames[j]) continue;
      q[j] = i < joints.position.size() ? joints.position[i] : 0.0;
      dq[j] = i < joints.velocity.size() ? joints.velocity[i] : 0.0;
      cur[j] = i < joints.effort.size() ? joints.effort[i] : 0.0;
    }
  }
  std::array<double, 5> ee{};
  if (chain_) {
    const std::array<double, 4> q4{q[0], q[1], q[2], q[3]};
    const auto tcp = chain_->tcp(q4);
    ee = {tcp.x, tcp.y, tcp.z, chain_->pitch_down(q4), chain_->tool_yaw(q4)};
  }
  std::array<double, 3> target{}, bin_xyz{};
  try {
    if (target_odom) {
      const auto t = tf_buffer_.lookup_transform("base_link", "odom", now() - 0.05);
      const auto p = srf::tfmath::apply(t.transform, *target_odom);
      target = {p.x, p.y, p.z};
    }
    if (const auto* marker = srf::nodes::find_marker(markers, bin)) {
      const auto t = tf_buffer_.lookup_transform("base_link", marker->frame_id.empty() ? "map" : marker->frame_id, now() - 0.05);
      const auto p = srf::tfmath::apply(t.transform, marker->pose.position);
      const double height = srf::nodes::marker_property_number(marker->properties, "height").value_or(0.10);
      bin_xyz = {p.x, p.y, height - kBaseLinkZ};
    }
  } catch (const std::exception& exception) {
    error = std::string("TF: ") + exception.what();
    return std::nullopt;
  }
  if (target_odom == std::nullopt) target = bin_xyz;  // place: 목표 = 바구니 (시범 데이터와 같다)
  char head[160];
  std::snprintf(head, sizeof(head), "{\"odom\":{\"v\":%.5f,\"w\":%.5f},\"scan_front_min\":%.4f,", v, w, front);
  return std::string(head) + "\"arm\":{\"position\":" + array_json(q.data(), 5) + ",\"velocity\":" +
         array_json(dq.data(), 5) + ",\"current\":" + array_json(cur.data(), 5) + "},\"ee\":" + array_json(ee.data(), 5) +
         ",\"target_xyz\":" + array_json(target.data(), 3) + ",\"bin_xyz\":" + array_json(bin_xyz.data(), 3) + "}";
}

std::optional<std::vector<VlaNode::Action>> VlaNode::request(const std::string& instruction,
                                                             const std::string& state, std::string& error) {
  srf::msgs::Image color, depth;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!color_seen_ || !depth_seen_) {
      error = !color_seen_ ? "카메라 color 가 없다" : "카메라 depth 가 없다";
      return std::nullopt;
    }
    color = color_;
    depth = depth_;
  }
  const std::string body = "{\"instruction\":" + json_quoted(instruction) + ",\"state\":" + state +
                           ",\"rgb\":{\"b64\":\"" + base64_encode(color.data) + "\",\"encoding\":" +
                           json_quoted(color.encoding) + ",\"width\":" + std::to_string(color.width) +
                           ",\"height\":" + std::to_string(color.height) + "},\"depth\":{\"b64\":\"" +
                           base64_encode(depth.data) + "\",\"encoding\":" + json_quoted(depth.encoding) +
                           ",\"width\":" + std::to_string(depth.width) + ",\"height\":" + std::to_string(depth.height) +
                           ",\"step\":" + std::to_string(depth.step) + "}}";
  const HttpResponse response = http_post_json(options_.url, body, options_.request_timeout_s);
  if (!response.ok) {
    error = "sidecar: " + response.error;
    return std::nullopt;
  }
  // "actions":[[8],[8],...]
  const auto at = response.body.find("\"actions\"");
  if (at == std::string::npos) {
    error = "sidecar 응답에 actions 가 없다";
    return std::nullopt;
  }
  std::vector<Action> actions;
  const char* p = response.body.c_str() + response.body.find('[', at) + 1;
  while (*p != '\0' && *p != ']') {
    while (*p == ' ' || *p == ',' || *p == '\n') ++p;
    if (*p != '[') break;
    ++p;
    Action a{};
    for (auto& value : a) {
      char* end = nullptr;
      value = std::strtod(p, &end);
      if (end == p) {
        error = "actions 파싱 실패";
        return std::nullopt;
      }
      p = end;
      while (*p == ',' || *p == ' ') ++p;
    }
    while (*p != '\0' && *p != ']') ++p;
    if (*p == ']') ++p;
    actions.push_back(a);
  }
  if (actions.empty()) {
    error = "actions 가 비었다";
    return std::nullopt;
  }
  return actions;
}

void VlaNode::apply(const Action& a, bool& buzzer_on, std::string& clipped) {
  const double v = std::clamp(a[0], -kMaxV, kMaxV), w = std::clamp(a[1], -kMaxW, kMaxW);
  if (v != a[0] || w != a[1]) clipped = "base";
  publish<srf::msgs::Twist>("cmd_vel_nav", srf::msgs::Twist{{v, 0.0, 0.0}, {0.0, 0.0, w}});
  srf::msgs::JointState command;
  command.header.stamp = now();
  command.name.assign(kJointNames.begin(), kJointNames.end());
  for (std::size_t j = 0; j < 5; ++j) {
    const double value = std::clamp(a[2 + j], kJointLimits[j][0], kJointLimits[j][1]);
    if (value != a[2 + j]) clipped = kJointNames[j];
    command.position.push_back(value);
  }
  publish<srf::msgs::JointState>("arm_joint_command", std::move(command));
  const bool on = a[7] > 0.5;
  if (on && !buzzer_on) {
    try {
      (void)context().services().call<srf::msgs::String, srf::msgs::TriggerResponse>("buzzer/melody",
                                                                                   srf::msgs::String{"beep"});
    } catch (const std::exception&) {
    }
  }
  buzzer_on = on;
}

srf::msgs::String VlaNode::execute(Handle& handle) {
  stop_.store(false);
  const std::string goal = handle.goal().data;
  const std::string kind = llm::string_field(goal, "kind").value_or("pick");
  const std::string instruction = llm::string_field(goal, "instruction").value_or("pick up the can");
  const std::string bin = llm::string_field(goal, "bin").value_or("");
  const auto feedback = [&](const std::string& json) { handle.publish_feedback(srf::msgs::String{json}); };
  const auto result = [&](bool ok, const std::string& extra) {
    stop_motion();
    std::string error;
    (void)switch_arm(false, error);
    return srf::msgs::String{"{\"ok\":" + std::string(ok ? "true" : "false") + ",\"kind\":" + json_quoted(kind) + extra + "}"};
  };
  // 대상(base_link) → odom 에 고정: 로봇이 다가가도 같은 물체를 가리킨다
  std::optional<srf::msgs::Point> target_odom;
  if (kind == "pick") {
    const auto xyz = vec3_field(goal, "target_xyz");
    if (!xyz) return result(false, ",\"error\":\"pick 에 target_xyz 가 없다\"");
    try {
      const auto t = tf_buffer_.lookup_transform("odom", "base_link", now() - 0.05);
      target_odom = srf::tfmath::apply(t.transform, srf::msgs::Point{(*xyz)[0], (*xyz)[1], (*xyz)[2]});
    } catch (const std::exception& exception) {
      return result(false, ",\"error\":" + json_quoted(std::string("TF odom<-base_link: ") + exception.what()));
    }
  }
  std::string error;
  if (!switch_arm(true, error)) return result(false, ",\"error\":" + json_quoted("컨트롤러 전환: " + error));

  const auto started = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration<double>(options_.period_s);
  int step = 0, still = 0, requests = 0;
  bool buzzer_on = false;
  std::string clipped_log;
  while (true) {
    if (handle.is_cancel_requested() || stop_.load()) return result(false, ",\"error\":\"취소\"");
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() > options_.timeout_s)
      return result(false, ",\"error\":\"시간 초과\",\"steps\":" + std::to_string(step));
    const auto state = state_json(target_odom, bin, error);
    if (!state) return result(false, ",\"error\":" + json_quoted(error));
    const auto chunk = request(instruction, *state, error);
    if (!chunk) return result(false, ",\"error\":" + json_quoted(error));
    ++requests;
    // 청크가 "멈춤"(바퀴 0, 관절·그리퍼 변화 없음)이면 할 일이 끝난 것
    bool static_chunk = true;
    for (const auto& a : *chunk) {
      if (std::abs(a[0]) > 0.01 || std::abs(a[1]) > 0.03) static_chunk = false;
      for (std::size_t j = 2; j < 6; ++j)
        if (std::abs(a[j] - chunk->front()[j]) > 0.02) static_chunk = false;
      if (std::abs(a[6] - chunk->front()[6]) > 0.002) static_chunk = false;
    }
    still = static_chunk ? still + 1 : 0;
    if (still >= 2) {
      return result(true, ",\"steps\":" + std::to_string(step) + ",\"requests\":" + std::to_string(requests) +
                              (clipped_log.empty() ? "" : ",\"clipped\":" + json_quoted(clipped_log)));
    }
    const std::size_t run = std::min(options_.replan_steps == 0 ? chunk->size() : options_.replan_steps, chunk->size());
    for (std::size_t k = 0; k < run; ++k) {
      if (handle.is_cancel_requested() || stop_.load()) return result(false, ",\"error\":\"취소\"");
      const auto tick = std::chrono::steady_clock::now();
      std::string clipped;
      apply((*chunk)[k], buzzer_on, clipped);
      if (!clipped.empty()) clipped_log = clipped;  // 마지막으로 범위 밖이었던 것
      char json[256];
      std::snprintf(json, sizeof(json),
                    "{\"kind\":\"%s\",\"step\":%d,\"v\":%.3f,\"w\":%.3f,\"gripper\":%.4f,\"buzzer\":%d%s%s%s}",
                    kind.c_str(), step, (*chunk)[k][0], (*chunk)[k][1], (*chunk)[k][6], (*chunk)[k][7] > 0.5 ? 1 : 0,
                    clipped.empty() ? "" : ",\"clipped\":\"", clipped.c_str(), clipped.empty() ? "" : "\"");
      feedback(json);
      ++step;
      std::this_thread::sleep_until(tick + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
    }
  }
}

bool VlaNode::switch_arm(bool manual, std::string& error) {
  srf::control::SwitchControllerRequest request;
  request.start_controllers = manual ? std::vector<std::string>{"arm_command_controller"}
                                     : std::vector<std::string>{"joint_trajectory_controller"};
  request.stop_controllers = manual ? std::vector<std::string>{"joint_trajectory_controller"}
                                    : std::vector<std::string>{"arm_command_controller"};
  request.strictness = 0;
  try {
    const auto response = context().services().call<srf::control::SwitchControllerRequest,
                                                    srf::control::SwitchControllerResponse>(
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

}  // namespace srf_recycling_ws
