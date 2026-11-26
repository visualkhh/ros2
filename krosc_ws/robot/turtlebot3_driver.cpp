#include "turtlebot3_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"

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

TurtleBot3Driver::TurtleBot3Driver(krosc::Context& context, RealTurtleBot3& robot)
    : Node(context, "TurtleBot3Driver"),
      robot_(robot),
      diagnostics_period_(declare_parameter<double>("diagnostics_period", 1.0,
                                                    [](double value) { return value > 0.0; })),
      data_timeout_(declare_parameter<double>("data_timeout", 2.0,
                                              [](double value) { return value > 0.0; })),
      arm_torque_on_start_(declare_parameter<bool>("arm_torque_on_start", true)) {
  last_diagnostics_ = now();
  last_arm_status_ = now();
  // 부팅 직후 토크가 꺼져 있으면 팔이 중력으로 떨어진다. 기본은 현재 자세를 잡게 한다.
  if (arm_torque_on_start_.value()) {
    robot_.opencr().set_arm_torque(true);
  }
  create_service<krosc::msgs::SetBoolRequest, krosc::msgs::SetBoolResponse>(
      "arm/set_torque", [this](const krosc::msgs::SetBoolRequest& request) {
        robot_.opencr().set_arm_torque(request.data);
        return krosc::msgs::SetBoolResponse{true, request.data ? "torque on" : "torque off"};
      });
  create_service<krosc::msgs::TriggerRequest, krosc::msgs::TriggerResponse>(
      "arm/stop", [this](const krosc::msgs::TriggerRequest&) {
        const ArmStatus arm = robot_.opencr().arm_status();
        if (!arm.present) {
          return krosc::msgs::TriggerResponse{false, "arm status 없음"};
        }
        robot_.opencr().write_arm(arm.position);
        return krosc::msgs::TriggerResponse{true, "현재 자세 유지"};
      });
  create_service<krosc::msgs::BuzzerRequest, krosc::msgs::BuzzerResponse>(
      "buzzer", [this](const krosc::msgs::BuzzerRequest& request) {
        if (request.frequency <= 0 || request.duration <= 0) {
          return krosc::msgs::BuzzerResponse{false, "frequency/duration은 0보다 커야 한다"};
        }
        robot_.opencr().buzzer(request.frequency, request.duration);
        return krosc::msgs::BuzzerResponse{true, "buzzer"};
      });
  create_service<krosc::msgs::ArmResetRequest, krosc::msgs::ArmResetResponse>(
      "arm/reset", [this](const krosc::msgs::ArmResetRequest& request) {
        std::array<bool, 5> mask{};
        const std::string text = request.target;
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
        return krosc::msgs::ArmResetResponse{true, "arm reset"};
      });
  logger().info("로봇 %s: OpenCR=%s, LDS-03=%s", robot_.name().c_str(),
                robot_.opencr().port().c_str(), robot_.has_lidar() ? robot_.lidar()->port().c_str() : "없음");
  logger().info("OpenCR IMU는 켠 뒤 2초 동안 가만히 둬야 자이로 편향을 잡는다");
}

void TurtleBot3Driver::poll() {
  const double current = now();
  if (current - last_diagnostics_ >= diagnostics_period_.value()) {
    publish_diagnostics();
    last_diagnostics_ = current;
  }
  if (current - last_arm_status_ >= 0.5) {
    publish_arm_status();
    last_arm_status_ = current;
  }
}

void TurtleBot3Driver::publish_arm_status() {
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
  text += ";pos=";
  for (std::size_t i = 0; i < arm.position.size(); ++i) {
    if (i != 0) {
      text += ",";
    }
    char buffer[24];
    if (!std::isfinite(arm.position[i])) {
      text += "x";
    } else {
      std::snprintf(buffer, sizeof(buffer), "%.3f", arm.position[i]);
      text += buffer;
    }
  }
  text += ";cur=";
  for (std::size_t i = 0; i < arm.current.size(); ++i) {
    if (i != 0) {
      text += ",";
    }
    char buffer[24];
    if (!std::isfinite(arm.current[i])) {
      text += "x";
    } else {
      std::snprintf(buffer, sizeof(buffer), "%.2f", arm.current[i]);
      text += buffer;
    }
  }
  publish("arm_status", krosc::msgs::String{text}, krosc::QOS_DEFAULT);
}

void TurtleBot3Driver::publish_diagnostics() {
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
