#pragma once

#include <string>

#include "krosc/msgs.hpp"

// nav2_msgs/action 흉내: 플래너/컨트롤러/BT가 액션으로 주고받는 Goal/Feedback/Result.
namespace krosc::msgs::nav2 {

struct ComputePathToPoseGoal {
  PoseStamped goal;
  PoseStamped start;
  std::string planner_id;
  bool use_start = false;
};
struct ComputePathToPoseFeedback {};
struct ComputePathToPoseResult {
  static constexpr int NONE = 0;
  static constexpr int UNKNOWN = 200;
  static constexpr int INVALID_PLANNER = 201;
  static constexpr int TF_ERROR = 202;
  static constexpr int START_OUTSIDE_MAP = 203;
  static constexpr int GOAL_OUTSIDE_MAP = 204;
  static constexpr int START_OCCUPIED = 205;
  static constexpr int GOAL_OCCUPIED = 206;
  static constexpr int TIMEOUT = 207;
  static constexpr int NO_VALID_PATH = 208;

  Path path;
  double planning_time = 0.0;
  int error_code = NONE;
  std::string error_msg;
};

struct FollowPathGoal {
  Path path;
  std::string controller_id;
  std::string goal_checker_id;
};
struct FollowPathFeedback {
  double distance_to_goal = 0.0;
  double speed = 0.0;
};
struct FollowPathResult {
  static constexpr int NONE = 0;
  static constexpr int UNKNOWN = 100;
  static constexpr int INVALID_CONTROLLER = 101;
  static constexpr int TF_ERROR = 102;
  static constexpr int INVALID_PATH = 103;
  static constexpr int PATIENCE_EXCEEDED = 104;
  static constexpr int FAILED_TO_MAKE_PROGRESS = 105;
  static constexpr int NO_VALID_CONTROL = 106;

  int error_code = NONE;
  std::string error_msg;
};

struct NavigateToPoseGoal {
  PoseStamped pose;
  std::string behavior_tree;
};
struct NavigateToPoseFeedback {
  PoseStamped current_pose;
  double navigation_time = 0.0;
  double distance_remaining = 0.0;
  int number_of_recoveries = 0;
};
struct NavigateToPoseResult {
  static constexpr int NONE = 0;
  static constexpr int UNKNOWN = 9000;
  static constexpr int FAILED_TO_LOAD_BEHAVIOR_TREE = 9001;
  static constexpr int TF_ERROR = 9002;
  static constexpr int TIMEOUT = 9003;

  int error_code = NONE;
  std::string error_msg;
};

struct SpinGoal {
  double target_yaw = 3.141592653589793;
  double time_allowance = 10.0;
};
struct SpinFeedback {
  double angular_distance_traveled = 0.0;
  double total_elapsed_time = 0.0;
};
struct SpinResult {
  static constexpr int NONE = 0;
  static constexpr int UNKNOWN = 400;
  static constexpr int TF_ERROR = 401;
  static constexpr int TIMEOUT = 402;

  int error_code = NONE;
  std::string error_msg;
};

struct BackUpGoal {
  double backup_dist = 0.3;
  double backup_speed = 0.1;
  double time_allowance = 10.0;
};
struct BackUpFeedback {
  double distance_traveled = 0.0;
  double total_elapsed_time = 0.0;
};
struct BackUpResult {
  static constexpr int NONE = 0;
  static constexpr int UNKNOWN = 410;
  static constexpr int TF_ERROR = 411;
  static constexpr int TIMEOUT = 412;
  static constexpr int COLLISION = 413;

  int error_code = NONE;
  std::string error_msg;
};

struct WaitGoal {
  double wait_duration = 1.0;
};
struct WaitFeedback {
  double time_left = 0.0;
};
struct WaitResult {
  static constexpr int NONE = 0;
  static constexpr int UNKNOWN = 420;

  int error_code = NONE;
  std::string error_msg;
};

inline constexpr const char* COMPUTE_PATH_TO_POSE_ACTION = "compute_path_to_pose";
inline constexpr const char* FOLLOW_PATH_ACTION = "follow_path";
inline constexpr const char* NAVIGATE_TO_POSE_ACTION = "navigate_to_pose";
inline constexpr const char* SPIN_ACTION = "spin";
inline constexpr const char* BACKUP_ACTION = "backup";
inline constexpr const char* WAIT_ACTION = "wait";

}  // namespace krosc::msgs::nav2
