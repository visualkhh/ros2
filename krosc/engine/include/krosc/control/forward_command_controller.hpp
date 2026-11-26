#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/control/controller.hpp"
#include "krosc/topic.hpp"

namespace krosc::control {

// control forward_command_controller 흉내: 토픽으로 받은 JointState를 그대로 command interface에 쓴다.
// krosc viz의 수동 팔 슬라이더(arm_joint_command)가 이 컨트롤러를 통해 하드웨어로 간다.
class ForwardCommandController final : public Controller {
 public:
  bool on_init(ControllerContext& context) override;
  bool on_activate() override;
  void update(double time, double period) override;

  [[nodiscard]] std::vector<std::string> command_interface_names() const override;
  [[nodiscard]] std::string type() const override {
    return "forward_command_controller/ForwardCommandController";
  }

 private:
  void on_command(const msgs::JointState& message);

  struct Joint {
    std::string name;
    std::shared_ptr<CommandInterface> command;
  };

  std::vector<Joint> joints_;
  Parameter<std::string> command_topic_;
  Subscription<msgs::JointState> subscription_;
  std::mutex mutex_;
  std::vector<double> pending_;
  bool has_pending_ = false;
};

}  // namespace krosc::control
