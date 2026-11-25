#include "krosc/net/rpc.hpp"

#include <any>
#include <cstdint>
#include <vector>

#include "krosc/action_types.hpp"
#include "krosc/msgs/control_msgs.hpp"
#include "krosc/msgs/moveit_msgs.hpp"
#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/net/rpc_codec.hpp"

#include "codec_util.hpp"

namespace krosc::msgs::codec {

namespace {

void encode_path(RpcWriter& w, const msgs::Path& path) {
  encode_header(w, path.header);
  w.u32(static_cast<std::uint32_t>(path.poses.size()));
  for (const auto& pose : path.poses) {
    encode_pose_stamped(w, pose);
  }
}

msgs::Path decode_path(RpcReader& r) {
  msgs::Path path;
  path.header = decode_header(r);
  const std::uint32_t count = r.u32();
  if (count > 1000000) {
    r.skip(r.remaining());
    return path;
  }
  path.poses.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    path.poses.push_back(decode_pose_stamped(r));
  }
  return path;
}

void encode_poses(RpcWriter& w, const std::vector<msgs::PoseStamped>& poses) {
  w.u32(static_cast<std::uint32_t>(poses.size()));
  for (const auto& pose : poses) {
    encode_pose_stamped(w, pose);
  }
}

std::vector<msgs::PoseStamped> decode_poses(RpcReader& r) {
  const std::uint32_t count = r.u32();
  if (count > 1000000) {
    r.skip(r.remaining());
    return {};
  }
  std::vector<msgs::PoseStamped> poses;
  poses.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    poses.push_back(decode_pose_stamped(r));
  }
  return poses;
}

}  // namespace

void register_action_rpc_codecs() {
  using net::ActionCodec;
  using net::RpcReader;
  using net::RpcWriter;

  net::register_rpc_codec<SendGoalResponse>(
      "krosc/action/SendGoalResponse",
      [](RpcWriter& w, const SendGoalResponse& value) { w.u8(value.accepted ? 1U : 0U); },
      [](RpcReader& r, SendGoalResponse& value) {
        value.accepted = r.u8() != 0;
        return true;
      });
  net::register_rpc_codec<CancelGoalResponse>(
      "krosc/action/CancelGoalResponse",
      [](RpcWriter& w, const CancelGoalResponse& value) { w.u8(value.canceled ? 1U : 0U); },
      [](RpcReader& r, CancelGoalResponse& value) {
        value.canceled = r.u8() != 0;
        return true;
      });

  ActionCodec move_group;
  move_group.name = "move_action";
  move_group.encode_goal = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* goal = std::any_cast<moveit::MoveGroupGoal>(&value);
    if (goal == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_pose_stamped(w, goal->target_pose);
    encode_doubles(w, goal->joint_goal);
    w.str(goal->planner_id);
    w.str(goal->controller);
    w.u8(goal->cartesian ? 1U : 0U);
    w.u8(goal->plan_only ? 1U : 0U);
    w.f64(goal->grip);
    w.str(goal->preset);
    return std::move(w.data);
  };
  move_group.decode_goal = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    moveit::MoveGroupGoal goal;
    goal.target_pose = decode_pose_stamped(r);
    goal.joint_goal = decode_doubles(r);
    goal.planner_id = r.str();
    goal.controller = r.str();
    goal.cartesian = r.u8() != 0;
    goal.plan_only = r.u8() != 0;
    goal.grip = r.f64();
    goal.preset = r.str();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(goal));
  };
  move_group.encode_feedback = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* feedback = std::any_cast<moveit::MoveGroupFeedback>(&value);
    if (feedback == nullptr) {
      return {};
    }
    RpcWriter w;
    w.f64(feedback->progress);
    w.str(feedback->state);
    return std::move(w.data);
  };
  move_group.decode_feedback = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    moveit::MoveGroupFeedback feedback;
    feedback.progress = r.f64();
    feedback.state = r.str();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(feedback));
  };
  move_group.encode_result = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* result = std::any_cast<moveit::MoveGroupResult>(&value);
    if (result == nullptr) {
      return {};
    }
    RpcWriter w;
    w.u32(static_cast<std::uint32_t>(result->error_code));
    w.str(result->error_msg);
    encode_joint_trajectory(w, result->trajectory);
    return std::move(w.data);
  };
  move_group.decode_result = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    moveit::MoveGroupResult result;
    result.error_code = static_cast<int>(r.u32());
    result.error_msg = r.str();
    result.trajectory = decode_joint_trajectory(r);
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(result));
  };
  net::register_action_codec(std::move(move_group));

  ActionCodec follow;
  follow.name = "follow_joint_trajectory";
  follow.encode_goal = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* goal = std::any_cast<control::FollowJointTrajectoryGoal>(&value);
    if (goal == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_joint_trajectory(w, goal->trajectory);
    encode_tolerances(w, goal->path_tolerance);
    encode_tolerances(w, goal->goal_tolerance);
    w.f64(goal->goal_time_tolerance);
    return std::move(w.data);
  };
  follow.decode_goal = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    control::FollowJointTrajectoryGoal goal;
    goal.trajectory = decode_joint_trajectory(r);
    goal.path_tolerance = decode_tolerances(r);
    goal.goal_tolerance = decode_tolerances(r);
    goal.goal_time_tolerance = r.f64();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(goal));
  };
  follow.encode_feedback = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* feedback = std::any_cast<control::FollowJointTrajectoryFeedback>(&value);
    if (feedback == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_header(w, feedback->header);
    encode_strings(w, feedback->joint_names);
    encode_joint_trajectory_point(w, feedback->desired);
    encode_joint_trajectory_point(w, feedback->actual);
    encode_joint_trajectory_point(w, feedback->error);
    return std::move(w.data);
  };
  follow.decode_feedback = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    control::FollowJointTrajectoryFeedback feedback;
    feedback.header = decode_header(r);
    feedback.joint_names = decode_strings(r);
    feedback.desired = decode_joint_trajectory_point(r);
    feedback.actual = decode_joint_trajectory_point(r);
    feedback.error = decode_joint_trajectory_point(r);
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(feedback));
  };
  follow.encode_result = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* result = std::any_cast<control::FollowJointTrajectoryResult>(&value);
    if (result == nullptr) {
      return {};
    }
    RpcWriter w;
    w.u32(static_cast<std::uint32_t>(result->error_code));
    w.str(result->error_string);
    return std::move(w.data);
  };
  follow.decode_result = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    control::FollowJointTrajectoryResult result;
    result.error_code = static_cast<int>(r.u32());
    result.error_string = r.str();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(result));
  };
  net::register_action_codec(std::move(follow));

  ActionCodec navigate_to_pose;
  navigate_to_pose.name = nav2::NAVIGATE_TO_POSE_ACTION;
  navigate_to_pose.encode_goal = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* goal = std::any_cast<nav2::NavigateToPoseGoal>(&value);
    if (goal == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_pose_stamped(w, goal->pose);
    w.str(goal->behavior_tree);
    return std::move(w.data);
  };
  navigate_to_pose.decode_goal = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    nav2::NavigateToPoseGoal goal;
    goal.pose = decode_pose_stamped(r);
    goal.behavior_tree = r.str();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(goal));
  };
  navigate_to_pose.encode_feedback = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* feedback = std::any_cast<nav2::NavigateToPoseFeedback>(&value);
    if (feedback == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_pose_stamped(w, feedback->current_pose);
    w.f64(feedback->navigation_time);
    w.f64(feedback->distance_remaining);
    w.u32(static_cast<std::uint32_t>(feedback->number_of_recoveries));
    return std::move(w.data);
  };
  navigate_to_pose.decode_feedback = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    nav2::NavigateToPoseFeedback feedback;
    feedback.current_pose = decode_pose_stamped(r);
    feedback.navigation_time = r.f64();
    feedback.distance_remaining = r.f64();
    feedback.number_of_recoveries = static_cast<int>(r.u32());
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(feedback));
  };
  navigate_to_pose.encode_result = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* result = std::any_cast<nav2::NavigateToPoseResult>(&value);
    if (result == nullptr) {
      return {};
    }
    RpcWriter w;
    w.u32(static_cast<std::uint32_t>(result->error_code));
    w.str(result->error_msg);
    return std::move(w.data);
  };
  navigate_to_pose.decode_result = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    nav2::NavigateToPoseResult result;
    result.error_code = static_cast<int>(r.u32());
    result.error_msg = r.str();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(result));
  };
  net::register_action_codec(std::move(navigate_to_pose));

  ActionCodec compute_path_to_pose;
  compute_path_to_pose.name = nav2::COMPUTE_PATH_TO_POSE_ACTION;
  compute_path_to_pose.encode_goal = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* goal = std::any_cast<nav2::ComputePathToPoseGoal>(&value);
    if (goal == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_pose_stamped(w, goal->goal);
    encode_pose_stamped(w, goal->start);
    w.str(goal->planner_id);
    w.u8(goal->use_start ? 1U : 0U);
    return std::move(w.data);
  };
  compute_path_to_pose.decode_goal = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    nav2::ComputePathToPoseGoal goal;
    goal.goal = decode_pose_stamped(r);
    goal.start = decode_pose_stamped(r);
    goal.planner_id = r.str();
    goal.use_start = r.u8() != 0;
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(goal));
  };
  compute_path_to_pose.encode_feedback = [](const std::any&) -> std::vector<std::uint8_t> {
    return {};
  };
  compute_path_to_pose.decode_feedback = [](const std::uint8_t*, std::size_t) -> std::any {
    return std::any(nav2::ComputePathToPoseFeedback{});
  };
  compute_path_to_pose.encode_result = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* result = std::any_cast<nav2::ComputePathToPoseResult>(&value);
    if (result == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_path(w, result->path);
    w.f64(result->planning_time);
    w.u32(static_cast<std::uint32_t>(result->error_code));
    w.str(result->error_msg);
    return std::move(w.data);
  };
  compute_path_to_pose.decode_result = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    nav2::ComputePathToPoseResult result;
    result.path = decode_path(r);
    result.planning_time = r.f64();
    result.error_code = static_cast<int>(r.u32());
    result.error_msg = r.str();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(result));
  };
  net::register_action_codec(std::move(compute_path_to_pose));

  ActionCodec compute_path_through_poses;
  compute_path_through_poses.name = nav2::COMPUTE_PATH_THROUGH_POSES_ACTION;
  compute_path_through_poses.encode_goal = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* goal = std::any_cast<nav2::ComputePathThroughPosesGoal>(&value);
    if (goal == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_poses(w, goal->goals);
    encode_pose_stamped(w, goal->start);
    w.str(goal->planner_id);
    w.u8(goal->use_start ? 1U : 0U);
    return std::move(w.data);
  };
  compute_path_through_poses.decode_goal = [](const std::uint8_t* data,
                                              std::size_t size) -> std::any {
    RpcReader r(data, size);
    nav2::ComputePathThroughPosesGoal goal;
    goal.goals = decode_poses(r);
    goal.start = decode_pose_stamped(r);
    goal.planner_id = r.str();
    goal.use_start = r.u8() != 0;
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(goal));
  };
  compute_path_through_poses.encode_feedback = [](const std::any&) -> std::vector<std::uint8_t> {
    return {};
  };
  compute_path_through_poses.decode_feedback = [](const std::uint8_t*, std::size_t) -> std::any {
    return std::any(nav2::ComputePathThroughPosesFeedback{});
  };
  compute_path_through_poses.encode_result = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* result = std::any_cast<nav2::ComputePathThroughPosesResult>(&value);
    if (result == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_path(w, result->path);
    w.f64(result->planning_time);
    w.u32(static_cast<std::uint32_t>(result->error_code));
    w.str(result->error_msg);
    return std::move(w.data);
  };
  compute_path_through_poses.decode_result = [](const std::uint8_t* data,
                                                std::size_t size) -> std::any {
    RpcReader r(data, size);
    nav2::ComputePathThroughPosesResult result;
    result.path = decode_path(r);
    result.planning_time = r.f64();
    result.error_code = static_cast<int>(r.u32());
    result.error_msg = r.str();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(result));
  };
  net::register_action_codec(std::move(compute_path_through_poses));

  ActionCodec navigate_through_poses;
  navigate_through_poses.name = nav2::NAVIGATE_THROUGH_POSES_ACTION;
  navigate_through_poses.encode_goal = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* goal = std::any_cast<nav2::NavigateThroughPosesGoal>(&value);
    if (goal == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_poses(w, goal->poses);
    w.str(goal->behavior_tree);
    return std::move(w.data);
  };
  navigate_through_poses.decode_goal = [](const std::uint8_t* data, std::size_t size) -> std::any {
    RpcReader r(data, size);
    nav2::NavigateThroughPosesGoal goal;
    goal.poses = decode_poses(r);
    goal.behavior_tree = r.str();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(goal));
  };
  navigate_through_poses.encode_feedback = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* feedback = std::any_cast<nav2::NavigateThroughPosesFeedback>(&value);
    if (feedback == nullptr) {
      return {};
    }
    RpcWriter w;
    encode_pose_stamped(w, feedback->current_pose);
    w.f64(feedback->navigation_time);
    w.f64(feedback->distance_remaining);
    w.u32(static_cast<std::uint32_t>(feedback->number_of_recoveries));
    w.u32(feedback->current_waypoint);
    return std::move(w.data);
  };
  navigate_through_poses.decode_feedback = [](const std::uint8_t* data,
                                              std::size_t size) -> std::any {
    RpcReader r(data, size);
    nav2::NavigateThroughPosesFeedback feedback;
    feedback.current_pose = decode_pose_stamped(r);
    feedback.navigation_time = r.f64();
    feedback.distance_remaining = r.f64();
    feedback.number_of_recoveries = static_cast<int>(r.u32());
    feedback.current_waypoint = r.u32();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(feedback));
  };
  navigate_through_poses.encode_result = [](const std::any& value) -> std::vector<std::uint8_t> {
    const auto* result = std::any_cast<nav2::NavigateThroughPosesResult>(&value);
    if (result == nullptr) {
      return {};
    }
    RpcWriter w;
    w.u32(static_cast<std::uint32_t>(result->error_code));
    w.str(result->error_msg);
    return std::move(w.data);
  };
  navigate_through_poses.decode_result = [](const std::uint8_t* data,
                                            std::size_t size) -> std::any {
    RpcReader r(data, size);
    nav2::NavigateThroughPosesResult result;
    result.error_code = static_cast<int>(r.u32());
    result.error_msg = r.str();
    if (!r.ok()) {
      return {};
    }
    return std::any(std::move(result));
  };
  net::register_action_codec(std::move(navigate_through_poses));
}

}  // namespace krosc::msgs::codec
