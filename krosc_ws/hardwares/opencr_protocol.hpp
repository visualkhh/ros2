#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"

namespace krosc_ws {

// OpenCR 펌웨어(firmware/opencr/tb3_wheels)의 S 줄.
struct BoardStatus {
  double left_position = std::numeric_limits<double>::quiet_NaN();
  double left_velocity = std::numeric_limits<double>::quiet_NaN();
  double right_position = std::numeric_limits<double>::quiet_NaN();
  double right_velocity = std::numeric_limits<double>::quiet_NaN();
  double uptime_ms = std::numeric_limits<double>::quiet_NaN();
  double left_current = std::numeric_limits<double>::quiet_NaN();
  double right_current = std::numeric_limits<double>::quiet_NaN();
  double left_temperature = std::numeric_limits<double>::quiet_NaN();
  double right_temperature = std::numeric_limits<double>::quiet_NaN();
  double battery_voltage = std::numeric_limits<double>::quiet_NaN();
};

// "S,1,2,3" -> ("S", [1,2,3]). 숫자가 아닌 값이 있으면 nullopt (nan/inf는 숫자).
std::optional<std::pair<std::string, std::vector<double>>> split_line(const std::string& line);

std::optional<BoardStatus> parse_status(const std::vector<double>& values);
std::optional<krosc::msgs::Imu> parse_imu(const std::vector<double>& values);
std::optional<std::pair<int, int>> parse_errors(const std::vector<double>& values);
std::optional<bool> arm_connected(const std::vector<double>& values);

// C,<left rad/s>,<right rad/s>
std::string format_command(double left, double right);
std::string error_text(int code);

}  // namespace krosc_ws
