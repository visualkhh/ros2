#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"

#include "opencr_device.hpp"

namespace krosc_ws {

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
