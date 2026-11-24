#include "robot_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf_ros.hpp"

namespace krosc_ws {

namespace {

std::string number(double value) {
  if (!std::isfinite(value)) {
    return "nan";
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return buffer;
}

void add_value(krosc::msgs::DiagnosticStatus& status, const std::string& key, const std::string& value) {
  status.values.push_back(krosc::msgs::KeyValue{key, value});
}

}  // namespace

RobotBridge::RobotBridge(krosc::Context& context, RealTurtleBot3& robot)
    : Node(context, "RobotBridge"),
      robot_(robot),
      diagnostics_period_(declare_parameter<double>("diagnostics_period", 1.0,
                                                    [](double value) { return value > 0.0; })),
      data_timeout_(declare_parameter<double>("data_timeout", 2.0,
                                              [](double value) { return value > 0.0; })) {
  last_diagnostics_ = now();
  last_arm_status_ = now();
  command_subscription_ = subscribe<krosc::msgs::Twist>(
      "cmd_vel",
      [this](const krosc::msgs::Twist& twist) {
        logger().debug("cmd_vel: v=%.2f w=%.2f", twist.linear.x, twist.angular.z);
        robot_.opencr().write(twist);
      },
      krosc::QOS_COMMAND);
  arm_torque_subscription_ = subscribe<krosc::msgs::Bool>(
      "arm_torque", [this](const krosc::msgs::Bool& message) { robot_.opencr().set_arm_torque(message.data); },
      krosc::QOS_DEFAULT);
  arm_stop_subscription_ = subscribe<krosc::msgs::Empty>(
      "arm_stop",
      [this](const krosc::msgs::Empty&) {
        const ArmStatus arm = robot_.opencr().arm_status();
        if (arm.present) {
          robot_.opencr().write_arm(arm.position);
        }
      },
      krosc::QOS_DEFAULT);
  arm_reset_subscription_ = subscribe<krosc::msgs::String>(
      "arm_reset",
      [this](const krosc::msgs::String& message) {
        std::array<bool, 5> mask{};
        const std::string text = message.data;
        if (text.empty() || text == "all" || text == "*") {
          mask.fill(true);
        } else {
          std::stringstream stream(text);
          std::string token;
          while (std::getline(stream, token, ',')) {
            const std::size_t begin = token.find_first_not_of(" \t");
            const std::size_t end = token.find_last_not_of(" \t");
            if (begin == std::string::npos) {
              continue;
            }
            token = token.substr(begin, end - begin + 1);
            if (token == "all" || token == "*") {
              mask.fill(true);
            } else if (token == "joint1" || token == "1" || token == "11") {
              mask[0] = true;
            } else if (token == "joint2" || token == "2" || token == "12") {
              mask[1] = true;
            } else if (token == "joint3" || token == "3" || token == "13") {
              mask[2] = true;
            } else if (token == "joint4" || token == "4" || token == "14") {
              mask[3] = true;
            } else if (token == "gripper" || token == "grip" || token == "5" || token == "15") {
              mask[4] = true;
            }
          }
        }
        robot_.opencr().reboot_arm(mask);
        logger().info("팔 리셋(AR): j1=%d j2=%d j3=%d j4=%d grip=%d", mask[0], mask[1], mask[2],
                      mask[3], mask[4]);
      },
      krosc::QOS_DEFAULT);
  logger().info("로봇 %s: OpenCR=%s, LDS-03=%s", robot_.name().c_str(),
                robot_.opencr().port().c_str(), robot_.has_lidar() ? robot_.lidar()->port().c_str() : "없음");
  logger().info("OpenCR IMU는 켠 뒤 2초 동안 가만히 둬야 자이로 편향을 잡는다");
}

void RobotBridge::poll() {
  const double current = now();
  if (auto imu = robot_.opencr().consume_imu()) {
    imu->header.stamp = current;
    imu->header.frame_id = "imu_link";
    publish("imu", std::move(*imu), krosc::QOS_SENSOR_DATA);
  }
  if (auto scan = robot_.scan(current)) {
    publish("scan", std::move(*scan), krosc::QOS_SENSOR_DATA);
  }
  if (current - last_diagnostics_ >= diagnostics_period_.value()) {
    publish_diagnostics();
    last_diagnostics_ = current;
  }
  if (current - last_arm_status_ >= 0.5) {
    publish_arm_status();
    last_arm_status_ = current;
  }
}

void RobotBridge::publish_arm_status() {
  const ArmStatus arm = robot_.opencr().arm_status();
  std::string text = "present=";
  text += arm.present ? "1" : "0";
  text += ";err=";
  for (std::size_t i = 0; i < arm.error.size(); ++i) {
    if (i != 0) {
      text += ",";
    }
    if (arm.error[i] < 0) {
      text += "x";
    } else {
      text += std::to_string(arm.error[i]);
    }
  }
  text += ";temp=";
  for (std::size_t i = 0; i < arm.temperature.size(); ++i) {
    if (i != 0) {
      text += ",";
    }
    if (!std::isfinite(arm.temperature[i])) {
      text += "x";
    } else {
      text += std::to_string(static_cast<int>(std::lround(arm.temperature[i])));
    }
  }
  publish("arm_status", krosc::msgs::String{text}, krosc::QOS_DEFAULT);
}

void RobotBridge::publish_diagnostics() {
  krosc::msgs::DiagnosticArray array;
  array.header.stamp = now();

  const auto& opencr = robot_.opencr();
  const double status_age = opencr.status_age();
  const auto board = opencr.status();
  krosc::msgs::DiagnosticStatus drive;
  drive.name = "OpenCR";
  drive.hardware_id = robot_.name();
  if (!opencr.connected()) {
    drive.level = 2;
    drive.message = "OpenCR가 연결되지 않았다";
  } else if (!board || status_age > data_timeout_.value()) {
    drive.level = 1;
    drive.message = board ? "OpenCR S 줄이 오래됐다" : "OpenCR S 줄을 아직 못 받았다";
  } else {
    drive.level = 0;
    drive.message = "ok";
  }
  add_value(drive, "connected", opencr.connected() ? "true" : "false");
  add_value(drive, "port", opencr.port());
  add_value(drive, "bad_lines", std::to_string(opencr.bad_lines()));
  add_value(drive, "errors", std::to_string(opencr.error_count()));
  add_value(drive, "last_error", opencr.last_error().empty() ? "-" : opencr.last_error());
  add_value(drive, "status_age", number(status_age));
  if (board) {
    add_value(drive, "battery", number(board->battery_voltage));
    add_value(drive, "left_temperature", number(board->left_temperature));
    add_value(drive, "right_temperature", number(board->right_temperature));
    add_value(drive, "left_current", number(board->left_current));
    add_value(drive, "right_current", number(board->right_current));
    add_value(drive, "uptime_ms", number(board->uptime_ms));
  }
  array.status.push_back(std::move(drive));

  krosc::msgs::DiagnosticStatus lidar_status;
  lidar_status.name = "LDS-03";
  lidar_status.hardware_id = robot_.name();
  if (!robot_.has_lidar()) {
    lidar_status.level = 1;
    lidar_status.message = "라이다가 없다";
  } else {
    const auto& lidar = *robot_.lidar();
    const double scan_age = lidar.scan_age();
    if (!lidar.connected()) {
      lidar_status.level = 2;
      lidar_status.message = "LDS-03가 연결되지 않았다";
    } else if (scan_age > data_timeout_.value()) {
      lidar_status.level = 1;
      lidar_status.message = "스캔이 오래됐다";
    } else {
      lidar_status.level = 0;
      lidar_status.message = "ok";
    }
    add_value(lidar_status, "connected", lidar.connected() ? "true" : "false");
    add_value(lidar_status, "port", lidar.port());
    add_value(lidar_status, "scans", std::to_string(lidar.scans()));
    add_value(lidar_status, "bad_packets", std::to_string(lidar.bad_packets()));
    add_value(lidar_status, "scan_age", number(scan_age));
  }
  array.status.push_back(std::move(lidar_status));
  publish("diagnostics", std::move(array));
}

}  // namespace krosc_ws
