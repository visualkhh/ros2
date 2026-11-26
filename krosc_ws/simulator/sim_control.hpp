#pragma once

#include <map>
#include <memory>

#include "krosc/control/control.hpp"
#include "krosc/simulator/turtlebot3.hpp"

namespace krosc_ws::sim {

// 시뮬레이션 로봇의 control 논리 레이어 (실기 OpenCrControl의 sim 버전).
class SimControl final : public krosc::control::Control {
 public:
  SimControl(krosc::simulator::TurtleBot3& robot, double wheel_radius, double wheel_separation);

  bool read(double time, double period) override;
  bool write(double time, double period) override;
  std::vector<std::shared_ptr<krosc::control::StateInterface>> export_state_interfaces() override;
  std::vector<std::shared_ptr<krosc::control::CommandInterface>> export_command_interfaces() override;

 private:
  krosc::simulator::TurtleBot3& robot_;
  const double wheel_radius_;
  const double wheel_separation_;
  std::map<std::string, std::shared_ptr<krosc::control::StateInterface>> states_;
  std::map<std::string, std::shared_ptr<krosc::control::CommandInterface>> commands_;
};

}  // namespace krosc_ws::sim
