#pragma once

#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/msgs/control_msgs.hpp"

// moveit_msgs/action/MoveGroup의 축소판. MoveIt의 실제 메시지는 MotionPlanRequest 전체를 담지만,
// krosc에서는 target_pose / joint_goal / planner_id / controller / plan_only만 쓴다.
namespace krosc::msgs::moveit {

struct MoveGroupGoal {
  PoseStamped target_pose;
  std::vector<double> joint_goal;
  std::string planner_id;
  std::string controller;
  bool cartesian = true;
  bool plan_only = false;
};

struct MoveGroupFeedback {
  double progress = 0.0;
  std::string state;
};

struct MoveGroupResult {
  static constexpr int NONE = 0;
  static constexpr int UNKNOWN = 1;
  static constexpr int INVALID_GOAL = 2;
  static constexpr int PLANNING_FAILED = 3;
  static constexpr int CONTROLLER_FAILED = 4;
  static constexpr int CANCELLED = 5;

  int error_code = NONE;
  std::string error_msg;
  control::JointTrajectory trajectory;
};

inline constexpr const char* MOVE_ACTION = "move_action";

}  // namespace krosc::msgs::moveit
