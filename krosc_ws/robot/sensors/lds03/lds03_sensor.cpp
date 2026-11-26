#include "lds03_sensor.hpp"

#include <utility>

#include "hardwares/lds03/lds03_device.hpp"
#include "krosc/qos.hpp"

namespace krosc_ws {

Lds03Sensor::Lds03Sensor(krosc::Context& context, Lds03Device& device, std::string topic,
                         std::string frame_id)
    : context_(context), device_(device), topic_(std::move(topic)),
      frame_id_(std::move(frame_id)) {}

void Lds03Sensor::start() { device_.start(); }
void Lds03Sensor::stop() { device_.stop(); }
bool Lds03Sensor::connected() const { return device_.connected(); }

bool Lds03Sensor::read(double time, double period) {
  (void)time;
  (void)period;
  if (auto scan = device_.take_scan(context_.now())) {
    scan->header.frame_id = frame_id_;
    if (!topic_.empty()) {
      context_.topics().publish<krosc::msgs::LaserScan>(topic_, std::move(*scan),
                                                        krosc::QOS_SENSOR_DATA);
    }
  }
  return device_.connected() && device_.scan_age() <= 1.0;
}

}  // namespace krosc_ws
