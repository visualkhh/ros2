#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "krosc/context.hpp"
#include "krosc/msgs.hpp"
#include "krosc/sensor/sensor.hpp"
#include "krosc/simulator/turtlebot3.hpp"

namespace krosc_ws::sim {

class SimImuSensor final : public krosc::sensor::Sensor {
 public:
  SimImuSensor(krosc::Context& context, krosc::simulator::TurtleBot3& robot, std::string topic,
               std::string frame_id);
  bool read(double time, double period) override;

 private:
  std::string topic_;
  std::string frame_id_;
  std::mutex mutex_;
  std::optional<krosc::msgs::Imu> latest_;
};

class SimScanSensor final : public krosc::sensor::Sensor {
 public:
  SimScanSensor(krosc::Context& context, krosc::simulator::TurtleBot3& robot, std::string topic,
                std::string frame_id);
  bool read(double time, double period) override;

 private:
  std::string topic_;
  std::string frame_id_;
  std::mutex mutex_;
  std::optional<krosc::msgs::LaserScan> latest_;
};

}  // namespace krosc_ws::sim
