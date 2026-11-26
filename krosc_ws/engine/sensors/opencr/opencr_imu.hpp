#pragma once

#include <string>

#include "krosc/context.hpp"
#include "krosc/sensor/sensor.hpp"

namespace krosc_ws {

class OpenCrDevice;

// OpenCR의 sensor 논리 레이어 (IMU).
// 물리 장치(OpenCrSerial)는 OpenCrControl이 수명주기를 소유하고, 여기서는 최신 IMU를 읽어 발행만 한다.
class OpenCrImu final : public krosc::sensor::Sensor {
 public:
  OpenCrImu(krosc::Context& context, OpenCrDevice& opencr, std::string topic = "imu",
            std::string frame_id = "imu_link");

  [[nodiscard]] bool is_connected() const override;
  bool read(double time, double period) override;

 private:
  OpenCrDevice& opencr_;
  std::string topic_;
  std::string frame_id_;
};

}  // namespace krosc_ws
