#include "ai_context_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf/math.hpp"

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
                                   std::string color_topic, std::string depth_topic)
    : Node(context, "robot_context"),
      scan_topic_(std::move(scan_topic)),
      color_topic_(std::move(color_topic)),
      depth_topic_(std::move(depth_topic)) {
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
  diagnostics_subscription_ = subscribe<krosc::msgs::DiagnosticArray>(
      "diagnostics", [this](const krosc::msgs::DiagnosticArray& message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (const auto& status : message.status) {
          if (status.name.find("OpenCR") != std::string::npos) {
            opencr_message_ = status.message;
            battery_voltage_ = value_of(status, "battery", 0.0);
            battery_percent_ = battery_percent(battery_voltage_);
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
}

AiContextNode::~AiContextNode() = default;

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
    bool tf_ok = false;
    try {
      const auto transform = tf_buffer_.lookup_transform("map", "base_footprint",
                                                         current - 0.05);
      if (transform.header.frame_id == "map") {
        pose_x = transform.transform.translation.x;
        pose_y = transform.transform.translation.y;
        pose_yaw = yaw_of(transform.transform.rotation);
        frame = "map";
        tf_ok = true;
      }
    } catch (const std::exception&) {
      try {
        const auto transform = tf_buffer_.lookup_transform("odom", "base_footprint",
                                                           current - 0.05);
        pose_x = transform.transform.translation.x;
        pose_y = transform.transform.translation.y;
        pose_yaw = yaw_of(transform.transform.rotation);
        frame = "odom";
        tf_ok = true;
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
        << ",\"robot\":{\"footprint\":[0.266,0.266],\"height\":0.141,"
        << "\"arm_base\":[-0.092,0,0.091],\"arm_reach\":0.449,"
        << "\"camera_mount\":[0.1167547,0.0325,0.0775]}"
        << ",\"tf\":{\"map_base\":" << (tf_ok ? "true" : "false")
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
    out << "],\"gripper\":" << fixed(gripper_, 3) << "}"
        << ",\"wheels\":{\"ok\":" << (wheel_seen_ ? "true" : "false")
        << ",\"left_pos\":" << fixed(wheel_left_pos_, 3)
        << ",\"right_pos\":" << fixed(wheel_right_pos_, 3)
        << ",\"left_vel\":" << fixed(wheel_left_vel_, 3)
        << ",\"right_vel\":" << fixed(wheel_right_vel_, 3) << "}"
        << ",\"battery\":{\"voltage\":" << fixed(battery_voltage_, 2)
        << ",\"percent\":" << fixed(battery_percent_, 1)
        << ",\"opencr\":\"" << escape_json(opencr_message_) << "\"}"
        << "}";
  }
  publish<krosc::msgs::String>("robot_context",
                               krosc::msgs::String{out.str()}, krosc::QOS_DEFAULT);
}

}  // namespace krosc_ws
