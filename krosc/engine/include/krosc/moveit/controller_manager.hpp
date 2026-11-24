#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "krosc/action.hpp"
#include "krosc/msgs/control_msgs.hpp"
#include "krosc/node.hpp"

// MoveIt의 MoveItControllerManager 플러그인 흉내.
//   move_group이 만든 궤적을 ros2_control 컨트롤러(JTC)의 FollowJointTrajectory 액션으로 보낸다.
namespace krosc::moveit {

using FollowJointTrajectoryHandle =
    ClientGoalHandle<msgs::control::FollowJointTrajectoryGoal,
                     msgs::control::FollowJointTrajectoryFeedback,
                     msgs::control::FollowJointTrajectoryResult>;

class MoveItControllerManager {
 public:
  virtual ~MoveItControllerManager() = default;

  virtual bool initialize(Node& node, const std::string& name,
                          const std::vector<std::string>& joints) = 0;
  [[nodiscard]] virtual std::shared_ptr<FollowJointTrajectoryHandle> send(
      const msgs::control::JointTrajectory& trajectory) = 0;
  [[nodiscard]] virtual bool cancel(const std::shared_ptr<FollowJointTrajectoryHandle>& handle) = 0;
  [[nodiscard]] virtual std::string type() const = 0;
};

using ControllerManagerFactory = std::function<std::unique_ptr<MoveItControllerManager>()>;

class ControllerManagerRegistry {
 public:
  void add(const std::string& type, ControllerManagerFactory factory) {
    factories_[type] = std::move(factory);
  }
  [[nodiscard]] std::unique_ptr<MoveItControllerManager> create(const std::string& type) const {
    const auto it = factories_.find(type);
    return it == factories_.end() ? nullptr : it->second();
  }
  [[nodiscard]] bool contains(const std::string& type) const { return factories_.count(type) != 0; }
  [[nodiscard]] std::vector<std::string> names() const {
    std::vector<std::string> result;
    for (const auto& [type, factory] : factories_) {
      (void)factory;
      result.push_back(type);
    }
    return result;
  }

 private:
  std::map<std::string, ControllerManagerFactory> factories_;
};

// moveit_simple_controller_manager와 같은 역할: 정해진 액션 이름으로 궤적을 보낸다.
class SimpleControllerManager final : public MoveItControllerManager {
 public:
  bool initialize(Node& node, const std::string& name,
                  const std::vector<std::string>& joints) override {
    joints_ = joints;
    action_name_ = node.declare_parameter<std::string>(name + ".action",
                                                       "joint_trajectory_controller/follow_joint_trajectory");
    client_ = std::make_unique<ActionClient<msgs::control::FollowJointTrajectoryGoal,
                                            msgs::control::FollowJointTrajectoryFeedback,
                                            msgs::control::FollowJointTrajectoryResult>>(
        node.context(), action_name_.value());
    return true;
  }

  [[nodiscard]] std::shared_ptr<FollowJointTrajectoryHandle> send(
      const msgs::control::JointTrajectory& trajectory) override {
    try {
      msgs::control::FollowJointTrajectoryGoal goal;
      goal.trajectory = trajectory;
      goal.goal_time_tolerance = 0.5;
      return client_->send_goal(goal);
    } catch (const std::exception&) {
      return nullptr;
    }
  }

  [[nodiscard]] bool cancel(const std::shared_ptr<FollowJointTrajectoryHandle>& handle) override {
    return handle ? handle->cancel() : false;
  }

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

inline ControllerManagerRegistry builtin_controller_managers() {
  ControllerManagerRegistry registry;
  registry.add("krosc_moveit/SimpleControllerManager",
               [] { return std::make_unique<SimpleControllerManager>(); });
  return registry;
}

}  // namespace krosc::moveit
