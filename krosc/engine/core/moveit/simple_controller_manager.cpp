#include "krosc/moveit/simple_controller_manager.hpp"

namespace krosc::moveit {

bool SimpleControllerManager::initialize(Node& node, const std::string& name,
                                         const std::vector<std::string>& joints) {
  joints_ = joints;
  action_name_ = node.declare_parameter<std::string>(
      name + ".action", "joint_trajectory_controller/follow_joint_trajectory");
  client_ = std::make_unique<ActionClient<msgs::control::FollowJointTrajectoryGoal,
                                          msgs::control::FollowJointTrajectoryFeedback,
                                          msgs::control::FollowJointTrajectoryResult>>(
      node.context(), action_name_.value());
  return true;
}

std::shared_ptr<FollowJointTrajectoryHandle> SimpleControllerManager::send(
    const msgs::control::JointTrajectory& trajectory) {
  try {
    msgs::control::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance = 0.5;
    return client_->send_goal(goal);
  } catch (const std::exception&) {
    return nullptr;
  }
}

bool SimpleControllerManager::cancel(const std::shared_ptr<FollowJointTrajectoryHandle>& handle) {
  return handle ? handle->cancel() : false;
}

}  // namespace krosc::moveit
