#include "ai_context_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf/math.hpp"

#include "nodes/common/http_client.hpp"

namespace krosc_ws {

namespace {

std::string escape_json(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 8);
  for (const char character : value) {
    if (character == '"' || character == '\\') out.push_back('\\');
    if (character == '\n') { out += "\\n"; continue; }
    if (character == '\r') continue;
    out.push_back(character);
  }
  return out;
}

std::string fixed(double value, int digits = 3) {
  if (!std::isfinite(value)) {
    return "0";
  }
  char buffer[40];
  std::snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
  return buffer;
}

double yaw_of(const krosc::msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double battery_percent(double voltage) {
  if (!std::isfinite(voltage) || voltage <= 0.0) {
    return 0.0;
  }
  // 3S Li-ion: 9.6V(0%) ~ 12.6V(100%). 참고값.
  return std::clamp((voltage - 9.6) / 3.0 * 100.0, 0.0, 100.0);
}

std::string base_url(std::string url) {
  while (!url.empty() && url.back() == '/') {
    url.pop_back();
  }
  return url;
}

std::string health_url(const std::string& url) {
  const std::string base = base_url(url);
  const std::size_t slash = base.rfind('/');
  if (slash != std::string::npos) {
    const std::string tail = base.substr(slash);
    if (tail == "/query" || tail == "/plan") {
      return base.substr(0, slash) + "/health";
    }
  }
  return base + "/health";
}

std::string tags_url(const std::string& url) {
  return base_url(url) + "/api/tags";
}

std::vector<std::string> split(const std::string& text, char separator) {
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (start <= text.size()) {
    const std::size_t end = text.find(separator, start);
    parts.push_back(text.substr(start, end == std::string::npos ? std::string::npos
                                                                : end - start));
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return parts;
}

std::string field_value(const std::vector<std::string>& fields, const std::string& prefix) {
  for (const auto& field : fields) {
    if (field.rfind(prefix, 0) == 0) {
      return field.substr(prefix.size());
    }
  }
  return {};
}

std::vector<double> number_list(const std::string& text) {
  std::vector<double> values;
  for (const auto& item : split(text, ',')) {
    if (item.empty() || item == "x") {
      continue;
    }
    try {
      values.push_back(std::stod(item));
    } catch (const std::exception&) {
    }
  }
  return values;
}

std::vector<int> int_list(const std::string& text) {
  std::vector<int> values;
  for (const auto& item : split(text, ',')) {
    if (item.empty() || item == "x") {
      values.push_back(-1);
      continue;
    }
    try {
      values.push_back(std::stoi(item));
    } catch (const std::exception&) {
      values.push_back(-1);
    }
  }
  return values;
}

std::string health_detail(const HttpResponse& response) {
  if (!response.ok) {
    return response.error.empty() ? "연결 실패" : response.error;
  }
  const std::string& body = response.body;
  const std::size_t key = body.find("\"model\"");
  if (key == std::string::npos) {
    return "ok";
  }
  const std::size_t start = body.find('"', key + 7);
  if (start == std::string::npos) {
    return "ok";
  }
  const std::size_t end = body.find('"', start + 1);
  if (end == std::string::npos) {
    return "ok";
  }
  return body.substr(start + 1, end - start - 1);
}

double value_of(const krosc::msgs::DiagnosticStatus& status, const std::string& key,
                double fallback = 0.0) {
  for (const auto& value : status.values) {
    if (value.key == key) {
      try {
        return std::stod(value.value);
      } catch (const std::exception&) {
        return fallback;
      }
    }
  }
  return fallback;
}

}  // namespace

AiContextNode::AiContextNode(krosc::Context& context, std::string scan_topic,
                             std::string color_topic, std::string depth_topic,
                             RobotGeometry geometry, AiServerConfig servers)
    : Node(context, "robot_context"),
      scan_topic_(std::move(scan_topic)),
      color_topic_(std::move(color_topic)),
      depth_topic_(std::move(depth_topic)),
      geometry_(geometry),
      servers_(std::move(servers)) {
  tf_listener_ = std::make_unique<krosc::TransformListener>(tf_buffer_, *this);
  odom_subscription_ = subscribe<krosc::msgs::Odometry>(
      "odom", [this](const krosc::msgs::Odometry& message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        odom_x_ = message.pose.pose.position.x;
        odom_y_ = message.pose.pose.position.y;
        odom_yaw_ = yaw_of(message.pose.pose.orientation);
        odom_linear_ = message.twist.twist.linear.x;
        odom_angular_ = message.twist.twist.angular.z;
        odom_stamp_ = now();
        odom_seen_ = true;
      }, krosc::QOS_SENSOR_DATA);
  scan_subscription_ = subscribe<krosc::msgs::LaserScan>(
      scan_topic_, [this](const krosc::msgs::LaserScan& message) {
        double front = std::numeric_limits<double>::infinity();
        double left = front;
        double right = front;
        double rear = front;
        for (std::size_t index = 0; index < message.ranges.size(); ++index) {
          const float raw = message.ranges[index];
          if (!std::isfinite(raw) || raw < message.range_min || raw > message.range_max) {
            continue;
          }
          const double angle = message.angle_min +
                               static_cast<double>(index) * message.angle_increment;
          const double c = std::cos(angle);
          const double s = std::sin(angle);
          if (c > 0.90) front = std::min(front, static_cast<double>(raw));
          if (c < -0.90) rear = std::min(rear, static_cast<double>(raw));
          if (s > 0.90) left = std::min(left, static_cast<double>(raw));
          if (s < -0.90) right = std::min(right, static_cast<double>(raw));
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        scan_front_ = std::isfinite(front) ? front : 0.0;
        scan_left_ = std::isfinite(left) ? left : 0.0;
        scan_right_ = std::isfinite(right) ? right : 0.0;
        scan_rear_ = std::isfinite(rear) ? rear : 0.0;
        scan_stamp_ = now();
        scan_seen_ = true;
      }, krosc::QOS_SENSOR_DATA);
  joint_subscription_ = subscribe<krosc::msgs::JointState>(
      "joint_states", [this](const krosc::msgs::JointState& message) {
        std::vector<double> arm(4, 0.0);
        double gripper = 0.0;
        double wheel_left = 0.0, wheel_right = 0.0;
        double wheel_left_velocity = 0.0, wheel_right_velocity = 0.0;
        bool have_arm = false, have_gripper = false;
        bool have_wheel = false;
        for (std::size_t index = 0; index < message.name.size() &&
                                    index < message.position.size(); ++index) {
          const std::string& name = message.name[index];
          const double position = message.position[index];
          const double velocity = index < message.velocity.size()
                                      ? message.velocity[index] : 0.0;
          if (name == "joint1") { arm[0] = position; have_arm = true; }
          else if (name == "joint2") { arm[1] = position; have_arm = true; }
          else if (name == "joint3") { arm[2] = position; have_arm = true; }
          else if (name == "joint4") { arm[3] = position; have_arm = true; }
          else if (name == "gripper_left_joint") {
            gripper = position;
            have_gripper = true;
          } else if (name == "wheel_left_joint" || name.find("wheel_left") != std::string::npos) {
            wheel_left = position;
            wheel_left_velocity = velocity;
            have_wheel = true;
          } else if (name == "wheel_right_joint" || name.find("wheel_right") != std::string::npos) {
            wheel_right = position;
            wheel_right_velocity = velocity;
            have_wheel = true;
          }
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (have_arm) arm_joints_ = arm;
        gripper_ = gripper;
        if (have_gripper) joints_seen_ = true;
        if (have_wheel) {
          wheel_left_pos_ = wheel_left;
          wheel_right_pos_ = wheel_right;
          wheel_left_vel_ = wheel_left_velocity;
          wheel_right_vel_ = wheel_right_velocity;
          wheel_seen_ = true;
        }
        joints_stamp_ = now();
      }, krosc::QOS_DEFAULT);
  arm_status_subscription_ = subscribe<krosc::msgs::String>(
      "arm_status", [this](const krosc::msgs::String& message) {
        const auto fields = split(message.data, ';');
        std::vector<double> temperatures = number_list(field_value(fields, "temp="));
        std::vector<double> currents = number_list(field_value(fields, "cur="));
        std::vector<int> errors = int_list(field_value(fields, "err="));
        std::lock_guard<std::mutex> lock(state_mutex_);
        arm_present_ = field_value(fields, "present=") == "1";
        arm_temperature_ = std::move(temperatures);
        arm_current_ = std::move(currents);
        arm_error_ = std::move(errors);
        arm_status_stamp_ = now();
        arm_status_seen_ = true;
      }, krosc::QOS_DEFAULT);
  diagnostics_subscription_ = subscribe<krosc::msgs::DiagnosticArray>(
      "diagnostics", [this](const krosc::msgs::DiagnosticArray& message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (const auto& status : message.status) {
          if (status.name.find("OpenCR") != std::string::npos) {
            opencr_message_ = status.message;
            battery_voltage_ = value_of(status, "battery", 0.0);
            battery_percent_ = battery_percent(battery_voltage_);
            wheel_left_temperature_ = value_of(status, "left_temperature", 0.0);
            wheel_right_temperature_ = value_of(status, "right_temperature", 0.0);
            wheel_left_current_ = value_of(status, "left_current", 0.0);
            wheel_right_current_ = value_of(status, "right_current", 0.0);
          }
        }
      }, krosc::QOS_DEFAULT);
  color_subscription_ = subscribe<krosc::msgs::Image>(
      color_topic_, [this](const krosc::msgs::Image& message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        color_width_ = message.width;
        color_height_ = message.height;
        color_stamp_ = message.header.stamp > 0.0 ? message.header.stamp : now();
        color_seen_ = true;
      }, krosc::QOS_SENSOR_DATA);
  depth_subscription_ = subscribe<krosc::msgs::Image>(
      depth_topic_, [this](const krosc::msgs::Image& message) {
        std::vector<double> values;
        if (message.encoding == "16UC1" && message.width > 0 && message.height > 0) {
          const std::uint32_t step = message.step > 0 ? message.step : message.width * 2;
          const std::uint32_t x0 = message.width / 3;
          const std::uint32_t x1 = message.width * 2 / 3;
          const std::uint32_t y0 = message.height / 3;
          const std::uint32_t y1 = message.height * 2 / 3;
          for (std::uint32_t y = y0; y < y1; y += 2) {
            const std::uint8_t* row = message.data.data() + static_cast<std::size_t>(y) * step;
            for (std::uint32_t x = x0; x < x1; x += 2) {
              const std::uint16_t mm = static_cast<std::uint16_t>(row[x * 2]) |
                                       (static_cast<std::uint16_t>(row[x * 2 + 1]) << 8);
              if (mm > 50 && mm < 3000) {
                values.push_back(static_cast<double>(mm) / 1000.0);
              }
            }
          }
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        depth_width_ = message.width;
        depth_height_ = message.height;
        depth_stamp_ = message.header.stamp > 0.0 ? message.header.stamp : now();
        depth_seen_ = true;
        if (!values.empty()) {
          std::sort(values.begin(), values.end());
          depth_min_ = values.front();
          depth_median_ = values[values.size() / 2];
        }
      }, krosc::QOS_SENSOR_DATA);
  timer_ = create_timer(0.2, [this] { update(); });
  logger().info("AiContextNode: ready scan=%s color=%s depth=%s",
                scan_topic_.c_str(), color_topic_.c_str(), depth_topic_.c_str());
  logger().info("AiContextNode: geometry footprint=%.3fx%.3f height=%.3f "
                "arm_reach=%.3f source=%s",
                geometry_.footprint[0], geometry_.footprint[1], geometry_.footprint[2],
                geometry_.arm_reach, geometry_.from_urdf ? "urdf" : "fallback");
  health_thread_ = std::thread([this] { health_loop(); });
}

AiContextNode::~AiContextNode() {
  health_stop_.store(true);
  if (health_thread_.joinable()) {
    health_thread_.join();
  }
}

void AiContextNode::health_loop() {
  while (!health_stop_.load()) {
    probe_servers();
    for (int tick = 0; tick < 30 && !health_stop_.load(); ++tick) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

void AiContextNode::probe_servers() {
  AiServerHealth llm;
  llm.enabled = servers_.llm_enabled && !servers_.llm_url.empty();
  if (llm.enabled) {
    const HttpResponse response = http_get(tags_url(servers_.llm_url), 0.8);
    const bool model_found = response.ok && !servers_.llm_model.empty() &&
                             response.body.find(servers_.llm_model) != std::string::npos;
    llm.ok = model_found;
    llm.detail = model_found ? servers_.llm_model
                             : (response.ok ? "model not found" : health_detail(response));
  } else {
    llm.detail = "disabled";
  }
  llm.checked_at = now();

  AiServerHealth vlm;
  vlm.enabled = servers_.vlm_enabled && !servers_.vlm_url.empty();
  if (vlm.enabled) {
    const HttpResponse response = http_get(health_url(servers_.vlm_url), 0.8);
    vlm.ok = response.ok;
    vlm.detail = health_detail(response);
  } else {
    vlm.detail = "disabled";
  }
  vlm.checked_at = now();

  AiServerHealth vla;
  vla.enabled = servers_.vla_enabled && !servers_.vla_url.empty();
  if (vla.enabled) {
    const HttpResponse response = http_get(health_url(servers_.vla_url), 0.8);
    vla.ok = response.ok;
    vla.detail = health_detail(response);
  } else {
    vla.detail = "disabled";
  }
  vla.checked_at = now();

  std::lock_guard<std::mutex> lock(health_mutex_);
  llm_health_ = std::move(llm);
  vlm_health_ = std::move(vlm);
  vla_health_ = std::move(vla);
}

void AiContextNode::update() {
  const double current = now();
  std::ostringstream out;
  out << "{\"stamp\":" << fixed(current, 3);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    double pose_x = odom_x_;
    double pose_y = odom_y_;
    double pose_yaw = odom_yaw_;
    std::string frame = "odom";
    bool map_ok = false;
    bool odom_ok = false;
    try {
      const auto transform = tf_buffer_.lookup_transform("map", "base_footprint",
                                                         current - 0.05);
      if (transform.header.frame_id == "map") {
        pose_x = transform.transform.translation.x;
        pose_y = transform.transform.translation.y;
        pose_yaw = yaw_of(transform.transform.rotation);
        frame = "map";
        map_ok = true;
      }
    } catch (const std::exception&) {
    }
    if (!map_ok) {
      try {
        const auto transform = tf_buffer_.lookup_transform("odom", "base_footprint",
                                                           current - 0.05);
        pose_x = transform.transform.translation.x;
        pose_y = transform.transform.translation.y;
        pose_yaw = yaw_of(transform.transform.rotation);
        frame = "odom";
        odom_ok = true;
      } catch (const std::exception&) {
      }
    }

    double camera_x = 0.0, camera_y = 0.0, camera_z = 0.0;
    double tcp_x = 0.0, tcp_y = 0.0, tcp_z = 0.0;
    bool camera_tf = false, tcp_tf = false;
    try {
      const auto transform = tf_buffer_.lookup_transform(
          "base_link", "camera_color_optical_frame", current - 0.05);
      camera_x = transform.transform.translation.x;
      camera_y = transform.transform.translation.y;
      camera_z = transform.transform.translation.z;
      camera_tf = true;
    } catch (const std::exception&) {
    }
    try {
      const auto transform = tf_buffer_.lookup_transform(
          "base_link", "end_effector_link", current - 0.05);
      tcp_x = transform.transform.translation.x;
      tcp_y = transform.transform.translation.y;
      tcp_z = transform.transform.translation.z;
      tcp_tf = true;
    } catch (const std::exception&) {
    }

    out << ",\"pose\":{\"frame\":\"" << frame << "\",\"x\":" << fixed(pose_x)
        << ",\"y\":" << fixed(pose_y) << ",\"yaw\":" << fixed(pose_yaw)
        << "},\"velocity\":{\"linear\":" << fixed(odom_linear_)
        << ",\"angular\":" << fixed(odom_angular_) << "}"
        << ",\"robot\":{\"footprint\":[" << fixed(geometry_.footprint[0], 3) << ","
        << fixed(geometry_.footprint[1], 3) << "],\"height\":"
        << fixed(geometry_.footprint[2], 3) << ",\"arm_base\":["
        << fixed(geometry_.arm_base[0], 3) << "," << fixed(geometry_.arm_base[1], 3)
        << "," << fixed(geometry_.arm_base[2], 3) << "],\"arm_reach\":"
        << fixed(geometry_.arm_reach, 3) << ",\"from_urdf\":"
        << (geometry_.from_urdf ? "true" : "false") << "}"
        << ",\"tf\":{\"map_base\":" << (map_ok ? "true" : "false")
        << ",\"odom_base\":" << (odom_ok ? "true" : "false")
        << ",\"camera_link\":" << (camera_tf ? "true" : "false")
        << ",\"tcp_link\":" << (tcp_tf ? "true" : "false")
        << ",\"camera\":[" << fixed(camera_x) << "," << fixed(camera_y) << ","
        << fixed(camera_z) << "],\"tcp\":[" << fixed(tcp_x) << "," << fixed(tcp_y)
        << "," << fixed(tcp_z) << "]}"
        << ",\"lidar\":{\"ok\":" << (scan_seen_ ? "true" : "false")
        << ",\"age\":" << fixed(scan_stamp_ > 0.0 ? current - scan_stamp_ : 999.0, 2)
        << ",\"front\":" << fixed(scan_front_, 3) << ",\"left\":" << fixed(scan_left_, 3)
        << ",\"right\":" << fixed(scan_right_, 3) << ",\"rear\":" << fixed(scan_rear_, 3)
        << "}"
        << ",\"camera\":{\"color_ok\":" << (color_seen_ ? "true" : "false")
        << ",\"color\":\"" << color_width_ << "x" << color_height_ << "\""
        << ",\"color_age\":" << fixed(color_stamp_ > 0.0 ? current - color_stamp_ : 999.0, 2)
        << ",\"depth_ok\":" << (depth_seen_ ? "true" : "false")
        << ",\"depth\":\"" << depth_width_ << "x" << depth_height_ << "\""
        << ",\"depth_age\":" << fixed(depth_stamp_ > 0.0 ? current - depth_stamp_ : 999.0, 2)
        << ",\"depth_min\":" << fixed(depth_min_, 3)
        << ",\"depth_median\":" << fixed(depth_median_, 3) << "}"
        << ",\"arm\":{\"ok\":" << (joints_seen_ ? "true" : "false")
        << ",\"joints\":[";
    for (std::size_t index = 0; index < arm_joints_.size(); ++index) {
      if (index != 0) out << ",";
      out << fixed(arm_joints_[index], 3);
    }
    out << "],\"gripper\":" << fixed(gripper_, 3)
        << ",\"present\":" << (arm_present_ ? "true" : "false")
        << ",\"status_age\":"
        << fixed(arm_status_stamp_ > 0.0 ? current - arm_status_stamp_ : 999.0, 1)
        << ",\"temperature\":[";
    for (std::size_t index = 0; index < arm_temperature_.size(); ++index) {
      if (index != 0) out << ",";
      out << fixed(arm_temperature_[index], 1);
    }
    out << "],\"current\":[";
    for (std::size_t index = 0; index < arm_current_.size(); ++index) {
      if (index != 0) out << ",";
      out << fixed(arm_current_[index], 2);
    }
    out << "],\"error\":[";
    for (std::size_t index = 0; index < arm_error_.size(); ++index) {
      if (index != 0) out << ",";
      out << arm_error_[index];
    }
    out << "]}"
        << ",\"wheels\":{\"ok\":" << (wheel_seen_ ? "true" : "false")
        << ",\"left_pos\":" << fixed(wheel_left_pos_, 3)
        << ",\"right_pos\":" << fixed(wheel_right_pos_, 3)
        << ",\"left_vel\":" << fixed(wheel_left_vel_, 3)
        << ",\"right_vel\":" << fixed(wheel_right_vel_, 3)
        << ",\"left_temperature\":" << fixed(wheel_left_temperature_, 1)
        << ",\"right_temperature\":" << fixed(wheel_right_temperature_, 1)
        << ",\"left_current\":" << fixed(wheel_left_current_, 2)
        << ",\"right_current\":" << fixed(wheel_right_current_, 2) << "}"
        << ",\"battery\":{\"voltage\":" << fixed(battery_voltage_, 2)
        << ",\"percent\":" << fixed(battery_percent_, 1)
        << ",\"opencr\":\"" << escape_json(opencr_message_) << "\"}";
    {
      std::lock_guard<std::mutex> health_lock(health_mutex_);
      const auto emit = [&](const char* name, const AiServerHealth& health) {
        out << "\"" << name << "\":{\"enabled\":" << (health.enabled ? "true" : "false")
            << ",\"ok\":" << (health.ok ? "true" : "false")
            << ",\"age\":" << fixed(health.checked_at > 0.0 ? current - health.checked_at : 999.0, 1)
            << ",\"detail\":\"" << escape_json(health.detail) << "\"}";
      };
      out << ",\"servers\":{";
      emit("llm", llm_health_);
      out << ",";
      emit("vlm", vlm_health_);
      out << ",";
      emit("vla", vla_health_);
      out << "}";
    }
    out << "}";
  }
  publish<krosc::msgs::String>("robot_context",
                               krosc::msgs::String{out.str()}, krosc::QOS_DEFAULT);
}

}  // namespace krosc_ws
