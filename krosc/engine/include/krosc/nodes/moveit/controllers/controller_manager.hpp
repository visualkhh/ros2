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
//   move_group이 만든 궤적을 control 컨트롤러(JTC)의 FollowJointTrajectory 액션으로 보낸다.
namespace krosc::moveit {

using FollowJointTrajectoryHandle =
    ClientGoalHandle<msgs::control::FollowJointTrajectoryGoal,
                     msgs::control::FollowJointTrajectoryFeedback,
                     msgs::control::FollowJointTrajectoryResult>;

class MoveItControllerManagerPlugin {
 public:
  virtual ~MoveItControllerManagerPlugin() = default;

  virtual bool initialize(Node& node, const std::string& name,
                          const std::vector<std::string>& joints) = 0;
  [[nodiscard]] virtual std::shared_ptr<FollowJointTrajectoryHandle> send(
      const msgs::control::JointTrajectory& trajectory) = 0;
  [[nodiscard]] virtual bool cancel(const std::shared_ptr<FollowJointTrajectoryHandle>& handle) = 0;
  [[nodiscard]] virtual std::string type() const = 0;
};

using ControllerManagerFactory = std::function<std::unique_ptr<MoveItControllerManagerPlugin>()>;

class ControllerManagerRegistry {
 public:
  void add(const std::string& type, ControllerManagerFactory factory) {
    factories_[type] = std::move(factory);
  }
  [[nodiscard]] std::unique_ptr<MoveItControllerManagerPlugin> create(const std::string& type) const {
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

ControllerManagerRegistry builtin_controller_managers();

}  // namespace krosc::moveit
