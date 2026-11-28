#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "srf/control/control.hpp"

#include "hardwares/opencr/opencr_device.hpp"

namespace srf_default_ws {

// OpenCR의 control 논리 레이어.
//   물리 장치(OpenCrSerial)에서 바퀴/팔 상태를 읽고(state), command interface를 명령으로 바꿔 쓴다.
//   HardwareManager가 read/write 주기를 정하고, write 끝에서 OpenCrSerial::tick()으로 실제 전송한다.
class OpenCrControl final : public srf::control::Control {
 public:
  OpenCrControl(OpenCrDevice& opencr, double wheel_radius, double wheel_separation);

  bool read(double time, double period) override;
  bool write(double time, double period) override;

  bool on_init(const srf::control::ControlInfo& info) override;
  std::vector<std::shared_ptr<srf::control::StateInterface>> export_state_interfaces() override;
  std::vector<std::shared_ptr<srf::control::CommandInterface>> export_command_interfaces() override;

 private:
  OpenCrDevice& opencr_;
  const double wheel_radius_;
  const double wheel_separation_;
  std::map<std::string, std::shared_ptr<srf::control::StateInterface>> states_;
  std::map<std::string, std::shared_ptr<srf::control::CommandInterface>> commands_;
  double last_write_time_ = -1e18;
  bool arm_ready_ = false;
};

}  // namespace srf_default_ws
