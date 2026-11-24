#pragma once

#include <string>
#include <vector>

#include "krosc/msgs.hpp"

// control_msgs/action/FollowJointTrajectory 흉내 (trajectory_msgs/JointTrajectory 포함).
namespace krosc::msgs::control {

struct JointTrajectoryPoint {
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  std::vector<double> effort;
  double time_from_start = 0.0;  // 초
};

struct JointTrajectory {
  Header header;
  std::vector<std::string> joint_names;
  std::vector<JointTrajectoryPoint> points;
};

struct JointTolerance {
  std::string name;
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
};

struct FollowJointTrajectoryGoal {
  JointTrajectory trajectory;
  std::vector<JointTolerance> path_tolerance;
  std::vector<JointTolerance> goal_tolerance;
  double goal_time_tolerance = 0.0;
};

struct FollowJointTrajectoryFeedback {
  Header header;
  std::vector<std::string> joint_names;
  JointTrajectoryPoint desired;
  JointTrajectoryPoint actual;
  JointTrajectoryPoint error;
};

struct FollowJointTrajectoryResult {
  static constexpr int SUCCESSFUL = 0;
  static constexpr int INVALID_GOAL = -1;
  static constexpr int INVALID_JOINTS = -2;
  static constexpr int OLD_HEADER_TIMESTAMP = -3;
  static constexpr int PATH_TOLERANCE_VIOLATED = -4;
  static constexpr int GOAL_TOLERANCE_VIOLATED = -5;
  static constexpr int INVALID_TRAJECTORY = -6;

  int error_code = SUCCESSFUL;
  std::string error_string;
};

// control_msgs/msg/JointTrajectoryControllerState 흉내 (viz/디버깅용).
struct JointTrajectoryControllerState {
  Header header;
  std::vector<std::string> joint_names;
  JointTrajectoryPoint desired;
  JointTrajectoryPoint actual;
  JointTrajectoryPoint error;
};

inline constexpr const char* FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory";

}  // namespace krosc::msgs::control
