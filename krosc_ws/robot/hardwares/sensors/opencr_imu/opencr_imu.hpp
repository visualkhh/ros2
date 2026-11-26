#pragma once

#include <string>

#include "krosc/context.hpp"
#include "krosc/sensor/sensor.hpp"

namespace krosc_ws {

class OpenCrSerial;

// OpenCR이 보내는 IMU를 SensorManager 주기에 맞춰 읽어 설정된 토픽으로 발행한다.
// 실제 시리얼 파싱은 OpenCrSerial이 하고, 여기서는 최신 IMU 한 개를 꺼내 publish만 한다.
class OpenCrImu final : public krosc::sensor::Sensor {
 public:
  OpenCrImu(krosc::Context& context, OpenCrSerial& opencr);

  bool on_init(const krosc::sensor::SensorInfo& info) override;
  bool on_update(double time, double period) override;

 private:
  OpenCrSerial& opencr_;
  std::string topic_ = "imu";
  std::string frame_id_ = "imu_link";
};

}  // namespace krosc_ws
