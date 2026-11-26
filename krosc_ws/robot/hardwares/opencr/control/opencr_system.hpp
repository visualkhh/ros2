#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "krosc/control/control_interface.hpp"

#include "real_turtlebot3.hpp"

namespace krosc_ws {

// control ControlInterface의 실기 구현.
//   OpenCR에서 바퀴 위치 + 팔 관절 위치를 읽고(state), 바퀴 속도/팔 위치 명령을 쓴다(command).
//   바퀴 속도는 diff_drive_controller, 팔 궤적은 joint_trajectory_controller가 command interface로 밀어 넣는다.
class OpenCrSystem final : public krosc::control::ControlInterface {
 public:
  explicit OpenCrSystem(RealTurtleBot3& robot);

  bool on_init(const krosc::control::ControlInfo& info) override;
  std::vector<std::shared_ptr<krosc::control::StateInterface>> export_state_interfaces() override;
  std::vector<std::shared_ptr<krosc::control::CommandInterface>> export_command_interfaces() override;
  bool read(double time, double period) override;
  bool write(double time, double period) override;

 private:
  RealTurtleBot3& robot_;
  std::map<std::string, std::shared_ptr<krosc::control::StateInterface>> states_;
  std::map<std::string, std::shared_ptr<krosc::control::CommandInterface>> commands_;
  double last_write_time_ = -1e18;
  bool arm_ready_ = false;
};

}  // namespace krosc_ws
