#include "opencr_imu.hpp"

#include <utility>

#include "hardwares/opencr/opencr_device.hpp"
#include "srf/qos.hpp"

namespace srf_default_ws {

OpenCrImu::OpenCrImu(srf::Context& context, OpenCrDevice& opencr, std::string topic,
                     std::string frame_id)
    : srf::sensor::Sensor(context), opencr_(opencr), topic_(std::move(topic)),
      frame_id_(std::move(frame_id)) {}

bool OpenCrImu::is_connected() const { return opencr_.is_connected(); }

bool OpenCrImu::read(double time, double period) {
  (void)time;
  (void)period;
  if (auto imu = opencr_.consume_imu()) {
    imu->header.stamp = now();
    imu->header.frame_id = frame_id_;
    if (!topic_.empty()) {
      context().topics().publish<srf::msgs::Imu>(topic_, std::move(*imu), srf::QOS_SENSOR_DATA);
    }
  }
  return opencr_.is_connected();
}

}  // namespace srf_default_ws
