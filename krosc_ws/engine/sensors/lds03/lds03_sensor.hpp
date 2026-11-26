#pragma once

#include <string>

#include "krosc/context.hpp"
#include "krosc/sensor/sensor.hpp"

namespace krosc_ws {

class LdsDevice;

// LDS-03의 sensor 논리 레이어. 물리 장치에서 스캔을 꺼내 토픽으로 발행한다.
class Lds03Sensor final : public krosc::sensor::Sensor {
 public:
  Lds03Sensor(krosc::Context& context, LdsDevice& device, std::string topic = "scan",
              std::string frame_id = "base_scan");

  [[nodiscard]] bool is_connected() const override;
  bool read(double time, double period) override;

 private:
  LdsDevice& device_;
  std::string topic_;
  std::string frame_id_;
};

}  // namespace krosc_ws
