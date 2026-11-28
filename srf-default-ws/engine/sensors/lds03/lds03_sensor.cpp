#include "lds03_sensor.hpp"

#include <utility>

#include "hardwares/lds03/lds_device.hpp"
#include "srf/qos.hpp"

namespace srf_default_ws {

Lds03Sensor::Lds03Sensor(srf::Context& context, LdsDevice& device, std::string topic,
                         std::string frame_id)
    : srf::sensor::Sensor(context), device_(device), topic_(std::move(topic)),
      frame_id_(std::move(frame_id)) {}

bool Lds03Sensor::is_connected() const { return device_.is_connected(); }

bool Lds03Sensor::read(double time, double period) {
  (void)time;
  (void)period;
  if (auto scan = device_.take_scan(now())) {
    scan->header.frame_id = frame_id_;
    if (!topic_.empty()) {
      context().topics().publish<srf::msgs::LaserScan>(topic_, std::move(*scan),
                                                        srf::QOS_SENSOR_DATA);
    }
  }
  return device_.is_connected() && device_.scan_age() <= 1.0;
}

}  // namespace srf_default_ws
