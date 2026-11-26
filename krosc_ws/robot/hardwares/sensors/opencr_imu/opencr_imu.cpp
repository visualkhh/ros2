#include "opencr_imu.hpp"

#include <utility>

#include "hardwares/opencr/opencr_serial.hpp"
#include "krosc/qos.hpp"

namespace krosc_ws {

OpenCrImu::OpenCrImu(krosc::Context& context, OpenCrSerial& opencr, std::string topic,
                     std::string frame_id)
    : krosc::sensor::Sensor(context), opencr_(opencr), topic_(std::move(topic)),
      frame_id_(std::move(frame_id)) {}

bool OpenCrImu::on_update(double time, double period) {
  (void)time;
  (void)period;
  if (auto imu = opencr_.consume_imu()) {
    imu->header.stamp = now();
    imu->header.frame_id = frame_id_;
    if (!topic_.empty()) {
      context().topics().publish<krosc::msgs::Imu>(topic_, std::move(*imu),
                                                   krosc::QOS_SENSOR_DATA);
    }
  }
  return opencr_.connected();
}

}  // namespace krosc_ws
