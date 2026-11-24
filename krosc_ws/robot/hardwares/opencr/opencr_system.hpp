#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "krosc/ros2_control/hardware_interface.hpp"

#include "real_turtlebot3.hpp"

namespace krosc_ws {

// ros2_control SystemInterface의 실기 구현.
//   OpenCR에서 바퀴 위치 + 팔 관절 위치를 읽고(state), 팔 위치 명령을 쓴다(command).
//   바퀴 속도 명령은 기존 cmd_vel -> OpenCR 경로를 그대로 쓴다 (ros2_control diff_drive_controller는 다음 단계).
class OpenCrSystem final : public krosc::ros2_control::SystemInterface {
 public:
  explicit OpenCrSystem(RealTurtleBot3& robot);

  bool on_init(const krosc::ros2_control::HardwareInfo& info) override;
  std::vector<std::shared_ptr<krosc::ros2_control::StateInterface>> export_state_interfaces() override;
  std::vector<std::shared_ptr<krosc::ros2_control::CommandInterface>> export_command_interfaces() override;
  bool read(double time, double period) override;
  bool write(double time, double period) override;

 private:
  RealTurtleBot3& robot_;
  std::map<std::string, std::shared_ptr<krosc::ros2_control::StateInterface>> states_;
  std::map<std::string, std::shared_ptr<krosc::ros2_control::CommandInterface>> commands_;
  double last_write_time_ = -1e18;
  bool arm_ready_ = false;
};

}  // namespace krosc_ws
