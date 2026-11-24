#pragma once

#include "krosc/action.hpp"
#include "krosc/nodes/moveit/controllers/controller_manager.hpp"
#include "krosc/msgs/control_msgs.hpp"

namespace krosc::moveit {

// moveit_simple_controller_manager와 같은 역할: 정해진 액션 이름으로 궤적을 보낸다.
class SimpleControllerManager final : public MoveItControllerManagerPlugin {
 public:
  bool initialize(Node& node, const std::string& name,
                  const std::vector<std::string>& joints) override;
  [[nodiscard]] std::shared_ptr<FollowJointTrajectoryHandle> send(
      const msgs::control::JointTrajectory& trajectory) override;
  [[nodiscard]] bool cancel(const std::shared_ptr<FollowJointTrajectoryHandle>& handle) override;
  [[nodiscard]] std::string type() const override {
    return "krosc_moveit/SimpleControllerManager";
  }

 private:
  Parameter<std::string> action_name_;
  std::vector<std::string> joints_;
  std::unique_ptr<ActionClient<msgs::control::FollowJointTrajectoryGoal,
                               msgs::control::FollowJointTrajectoryFeedback,
                               msgs::control::FollowJointTrajectoryResult>>
      client_;
};

}  // namespace krosc::moveit
