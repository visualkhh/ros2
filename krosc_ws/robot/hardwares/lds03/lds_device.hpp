#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "krosc/hardware/hardware.hpp"
#include "krosc/msgs.hpp"

namespace krosc_ws {

// 실기 Lds03Device와 시뮬레이터 LDS가 공유하는 장치 인터페이스.
class LdsDevice : public krosc::hardware::Hardware {
 public:
  [[nodiscard]] virtual std::optional<krosc::msgs::LaserScan> take_scan(double stamp) = 0;
  [[nodiscard]] virtual std::uint64_t scans() const = 0;
  [[nodiscard]] virtual std::uint64_t bad_packets() const = 0;
  [[nodiscard]] virtual double scan_age() const = 0;
  [[nodiscard]] virtual const std::string& port() const = 0;
};

}  // namespace krosc_ws
