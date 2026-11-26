#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "krosc/control/controller.hpp"

namespace krosc::control {

// control joint_state_broadcaster 흉내: state interface의 position을 모아 sensor_msgs/JointState로 낸다.
class JointStateBroadcaster final : public Controller {
 public:
  bool on_init(ControllerContext& context) override;
  bool on_activate() override;
  void update(double time, double period) override;

  [[nodiscard]] std::string type() const override {
    return "joint_state_broadcaster/JointStateBroadcaster";
  }

 private:
  std::vector<std::pair<std::string, std::shared_ptr<StateInterface>>> joints_;
  Parameter<std::vector<std::string>> joints_parameter_;
};

}  // namespace krosc::control
