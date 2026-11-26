#include "opencr_imu.hpp"

#include <utility>

#include "hardwares/controls/opencr/opencr_serial.hpp"
#include "krosc/qos.hpp"

namespace krosc_ws {

OpenCrImu::OpenCrImu(krosc::Context& context, OpenCrSerial& opencr)
    : krosc::sensor::Sensor(context), opencr_(opencr) {}

bool OpenCrImu::on_init(const krosc::sensor::SensorInfo& info) {
  if (!krosc::sensor::Sensor::on_init(info)) {
    return false;
  }
  if (!info.topic.empty()) {
    topic_ = info.topic;
  }
  if (!info.frame_id.empty()) {
    frame_id_ = info.frame_id;
  }
  return true;
}

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
