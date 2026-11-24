#pragma once

#include <string>

#include "krosc/ros2_control/hardware_interface.hpp"

namespace krosc::ros2_control {

// read-only 센서 하드웨어의 수명주기. ROS 2 hardware_interface의 SensorInterface에 해당하는
// krosc 축소판이다. LaserScan 같은 배열 메시지는 double state interface로 표현하지 않고
// 드라이버 노드가 토픽으로 내보낸다. 여기서는 on_init/activate/deactivate/read만 맞춘다.
class SensorInterface {
 public:
  virtual ~SensorInterface() = default;

  virtual bool on_init(const HardwareInfo& info) {
    info_ = info;
    return true;
  }
  virtual bool on_activate() { return true; }
  virtual bool on_deactivate() { return true; }
  virtual bool read(double time, double period) = 0;

  [[nodiscard]] const HardwareInfo& info() const { return info_; }

 private:
  HardwareInfo info_;
};

}  // namespace krosc::ros2_control
