#include "sim_sensors.hpp"

#include <utility>

#include "krosc/qos.hpp"

namespace krosc_ws::sim {

SimImuSensor::SimImuSensor(krosc::Context& context, krosc::simulator::TurtleBot3& robot,
                           std::string topic, std::string frame_id)
    : krosc::sensor::Sensor(context), topic_(std::move(topic)), frame_id_(std::move(frame_id)) {
  robot.set_imu_callback([this](const krosc::msgs::Imu& imu) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = imu;
  });
}

bool SimImuSensor::read(double, double) {
  std::optional<krosc::msgs::Imu> imu;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    imu = std::move(latest_);
    latest_.reset();
  }
  if (!imu) {
    return true;
  }
  imu->header.stamp = now();
  imu->header.frame_id = frame_id_;
  if (!topic_.empty()) {
    context().topics().publish<krosc::msgs::Imu>(topic_, std::move(*imu), krosc::QOS_SENSOR_DATA);
  }
  return true;
}

SimScanSensor::SimScanSensor(krosc::Context& context, krosc::simulator::TurtleBot3& robot,
                             std::string topic, std::string frame_id)
    : krosc::sensor::Sensor(context), topic_(std::move(topic)), frame_id_(std::move(frame_id)) {
  robot.set_scan_callback([this](const krosc::msgs::LaserScan& scan) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = scan;
  });
}

bool SimScanSensor::read(double, double) {
  std::optional<krosc::msgs::LaserScan> scan;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scan = std::move(latest_);
    latest_.reset();
  }
  if (!scan) {
    return true;
  }
  scan->header.stamp = now();
  scan->header.frame_id = frame_id_;
  if (!topic_.empty()) {
    context().topics().publish<krosc::msgs::LaserScan>(topic_, std::move(*scan),
                                                       krosc::QOS_SENSOR_DATA);
  }
  return true;
}

}  // namespace krosc_ws::sim
