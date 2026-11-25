#include "krosc/net/rpc.hpp"

#include <any>
#include <cstdint>
#include <vector>

#include "krosc/action_types.hpp"
#include "krosc/msgs/control_msgs.hpp"
#include "krosc/msgs/moveit_msgs.hpp"
#include "krosc/net/rpc_codec.hpp"

#include "codec_util.hpp"

namespace krosc::msgs::codec {

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
}

}  // namespace krosc::msgs::codec
