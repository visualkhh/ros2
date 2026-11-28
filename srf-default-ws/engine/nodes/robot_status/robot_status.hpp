#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "srf/node.hpp"
#include "srf/parameter.hpp"
#include "srf/topic.hpp"

namespace srf_default_ws {

class OpenCrDevice;
class LdsDevice;

// OpenCR/LDS-03의 논리 상태/서비스 노드.
// 센서 데이터는 HardwareManager가 발행하고, 여기서는 arm_status/diagnostics와
// 팔 토크/정지/리셋/부저 서비스를 담당한다.
class RobotStatus final : public srf::Node {
 public:
  RobotStatus(srf::Context& context, OpenCrDevice& opencr, LdsDevice* lidar);

  void poll();

 private:
  struct MelodyNote {
    int frequency_hz = 0;
    int duration_ms = 0;
  };

  void publish_diagnostics();
  void publish_arm_status();
  bool play_melody(const std::string& name);
  void melody_tick();

  OpenCrDevice& opencr_;
  LdsDevice* lidar_;
  srf::Parameter<double> diagnostics_period_;
  srf::Parameter<double> data_timeout_;
  srf::Parameter<bool> arm_torque_on_start_;
  srf::Timer poll_timer_;
  double last_diagnostics_ = 0.0;
  double last_arm_status_ = 0.0;

  std::mutex melody_mutex_;
  std::vector<MelodyNote> melody_;
  std::size_t melody_index_ = 0;
  double melody_next_ = 0.0;
  bool melody_active_ = false;
  srf::Timer melody_timer_;
};

}  // namespace srf_default_ws
