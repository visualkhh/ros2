#include "opencr_protocol.hpp"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <sstream>

namespace srf_default_ws {

namespace {
bool parse_double(const std::string& text, double& value) {
  char* end = nullptr;
  errno = 0;
  value = std::strtod(text.c_str(), &end);
  if (end == text.c_str()) {
    return false;
  }
  while (*end != '\0' && std::isspace(static_cast<unsigned char>(*end)) != 0) {
    ++end;
  }
  return *end == '\0';
}
}  // namespace

std::optional<std::pair<std::string, std::vector<double>>> split_line(const std::string& line) {
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (start <= line.size()) {
    const std::size_t comma = line.find(',', start);
    if (comma == std::string::npos) {
      fields.push_back(line.substr(start));
      break;
    }
    fields.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
  if (fields.empty() || fields[0].empty()) {
    return std::nullopt;
  }
  std::vector<double> values;
  values.reserve(fields.size() - 1);
  for (std::size_t i = 1; i < fields.size(); ++i) {
    double value = 0.0;
    if (!parse_double(fields[i], value)) {
      return std::nullopt;
    }
    values.push_back(value);
  }
  return std::make_pair(fields[0], std::move(values));
}

std::optional<BoardStatus> parse_status(const std::vector<double>& values) {
  if (values.size() < 5) {
    return std::nullopt;
  }
  BoardStatus status;
  const auto assign = [&](std::size_t index, double& target) {
    if (index < values.size()) {
      target = values[index];
    }
  };
  assign(0, status.left_position);
  assign(1, status.left_velocity);
  assign(2, status.right_position);
  assign(3, status.right_velocity);
  assign(4, status.uptime_ms);
  assign(5, status.left_current);
  assign(6, status.right_current);
  assign(7, status.left_temperature);
  assign(8, status.right_temperature);
  assign(9, status.battery_voltage);
  return status;
}

std::optional<srf::msgs::Imu> parse_imu(const std::vector<double>& values) {
  if (values.size() < 10) {
    return std::nullopt;
  }
  srf::msgs::Imu imu;
  imu.header.stamp = 0.0;
  imu.header.frame_id = "imu_link";
  imu.angular_velocity = {values[0], values[1], values[2]};
  imu.linear_acceleration = {values[3], values[4], values[5]};
  imu.orientation = {values[7], values[8], values[9], values[6]};  // qx, qy, qz, qw
  imu.orientation_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return imu;
}

std::optional<std::pair<int, int>> parse_errors(const std::vector<double>& values) {
  if (values.size() < 2 || !std::isfinite(values[0]) || !std::isfinite(values[1])) {
    return std::nullopt;
  }
  return std::make_pair(static_cast<int>(values[0]), static_cast<int>(values[1]));
}

std::optional<bool> arm_connected(const std::vector<double>& values) {
  constexpr std::size_t kArmValues = 18;
  if (values.size() < kArmValues) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < kArmValues; ++i) {
    if (!std::isnan(values[i])) {
      return true;
    }
  }
  return false;
}

std::string format_command(double left, double right) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "C,%.4f,%.4f", left, right);
  return buffer;
}

std::string error_text(int code) {
  if (code == 0) {
    return "정상";
  }
  if (code < 0) {
    return "무응답";
  }
  static constexpr std::pair<int, const char*> kBits[] = {
      {1, "입력 전압"}, {4, "과열"}, {8, "모터 엔코더"}, {16, "전기 충격"}, {32, "과부하"},
  };
  std::string text;
  for (const auto& [bit, name] : kBits) {
    if ((code & bit) != 0) {
      if (!text.empty()) {
        text += "+";
      }
      text += name;
    }
  }
  return text.empty() ? std::to_string(code) : text;
}

}  // namespace srf_default_ws
