#pragma once

#include <string>

#include "krosc/context.hpp"
#include "krosc/hardware/hardware.hpp"

namespace krosc_ws {

class OpenCrSerial;

// OpenCR의 sensor 논리 레이어 (IMU).
// 물리 장치(OpenCrSerial)는 OpenCrControl이 수명주기를 소유하고, 여기서는 최신 IMU를 읽어 발행만 한다.
class OpenCrImu final : public krosc::hardware::Hardware {
 public:
  OpenCrImu(krosc::Context& context, OpenCrSerial& opencr, std::string topic = "imu",
            std::string frame_id = "imu_link");

  void start() override {}
  void stop() override {}
  [[nodiscard]] bool connected() const override;
  bool read(double time, double period) override;

 private:
  krosc::Context& context_;
  OpenCrSerial& opencr_;
  std::string topic_;
  std::string frame_id_;
};

}  // namespace krosc_ws
