#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "srf/hardware/hardware.hpp"
#include "srf/msgs.hpp"

namespace srf_recycling_ws {

struct ArmStatus {
  std::array<double, 5> position{};
  std::array<double, 5> velocity{};
  std::array<double, 5> current{};
  std::array<double, 5> temperature{};
  std::array<int, 5> error{-1, -1, -1, -1, -1};
  bool present = false;
};

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

// 실기 OpenCrSerial과 시뮬레이터 OpenCR이 공유하는 장치 인터페이스.
class OpenCrDevice : public srf::hardware::Hardware {
 public:
  virtual void write(const srf::msgs::Twist& command) = 0;
  virtual void write_arm(const std::array<double, 5>& position) = 0;
  virtual void set_arm_torque(bool enabled) = 0;
  virtual void buzzer(int frequency_hz, int duration_ms) = 0;
  virtual void reboot_arm(const std::array<bool, 5>& joints) = 0;
  virtual void tick() = 0;

  [[nodiscard]] virtual std::optional<srf::msgs::Imu> consume_imu() = 0;
  [[nodiscard]] virtual ArmStatus arm_status() const = 0;

  [[nodiscard]] virtual double left_position() const = 0;
  [[nodiscard]] virtual double right_position() const = 0;
  [[nodiscard]] virtual double left_velocity() const = 0;
  [[nodiscard]] virtual double right_velocity() const = 0;

  [[nodiscard]] virtual std::optional<BoardStatus> status() const = 0;
  [[nodiscard]] virtual double status_age() const = 0;
  [[nodiscard]] virtual std::uint64_t error_count() const = 0;
  [[nodiscard]] virtual std::uint64_t bad_lines() const = 0;
  [[nodiscard]] virtual std::string last_error() const = 0;
  [[nodiscard]] virtual const std::string& port() const = 0;
};

}  // namespace srf_recycling_ws
