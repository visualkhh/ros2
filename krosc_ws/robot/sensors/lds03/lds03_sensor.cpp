#include "lds03_sensor.hpp"

#include <utility>

#include "hardwares/lds03/lds03_device.hpp"
#include "krosc/qos.hpp"

namespace krosc_ws {

Lds03Sensor::Lds03Sensor(krosc::Context& context, Lds03Device& device, std::string topic,
                         std::string frame_id)
    : krosc::sensor::Sensor(context), device_(device), topic_(std::move(topic)),
      frame_id_(std::move(frame_id)) {}

bool Lds03Sensor::connected() const { return device_.connected(); }

bool Lds03Sensor::read(double time, double period) {
  (void)time;
  (void)period;
  if (auto scan = device_.take_scan(now())) {
    scan->header.frame_id = frame_id_;
    if (!topic_.empty()) {
      context().topics().publish<krosc::msgs::LaserScan>(topic_, std::move(*scan),
                                                        krosc::QOS_SENSOR_DATA);
    }
  }
  return device_.connected() && device_.scan_age() <= 1.0;
}

}  // namespace krosc_ws
