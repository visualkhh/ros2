#pragma once

#include <string>

#include "krosc/context.hpp"
#include "krosc/hardware/hardware.hpp"

namespace krosc_ws {

class Lds03Device;

// LDS-03의 sensor 논리 레이어. 물리 장치에서 스캔을 꺼내 토픽으로 발행한다.
class Lds03Sensor final : public krosc::hardware::Hardware {
 public:
  Lds03Sensor(krosc::Context& context, Lds03Device& device, std::string topic = "scan",
              std::string frame_id = "base_scan");

  void start() override;
  void stop() override;
  [[nodiscard]] bool connected() const override;
  bool read(double time, double period) override;

 private:
  krosc::Context& context_;
  Lds03Device& device_;
  std::string topic_;
  std::string frame_id_;
};

}  // namespace krosc_ws
