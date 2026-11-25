#include "krosc/nodes/nav2/bt_navigator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <utility>

#include "krosc/tf/math.hpp"
#include "krosc/qos.hpp"

namespace krosc::nodes::nav2 {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr const char* kReplaced = "새 경로로 바뀌었다";

std::string status_name(GoalStatus status) {
  switch (status) {
    case GoalStatus::Accepted:
      return "accepted";
    case GoalStatus::Succeeded:
      return "succeeded";
    case GoalStatus::Canceled:
      return "canceled";
    case GoalStatus::Aborted:
      return "aborted";
    case GoalStatus::Rejected:
      return "rejected";
  }
  return "unknown";
}
}  // namespace

BtNavigator::BtNavigator(Context& context, std::string name,
                         std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      replanning_period_(declare_parameter<double>("replanning_period", 1.0,
                                                  [](double value) { return value > 0.0; })),
      number_of_recoveries_(declare_parameter<std::int64_t>(
          "number_of_recoveries", 3, [](std::int64_t value) { return value >= 0; })),
      recovery_backup_(declare_parameter<bool>("recovery_backup", true)),
      global_frame_(declare_parameter<std::string>("global_frame", "map")),
      robot_base_frame_(declare_parameter<std::string>("robot_base_frame", "base_footprint")),
      goal_topic_(declare_parameter<std::string>("goal_topic", "goal_pose")),
      goal_poses_topic_(declare_parameter<std::string>("goal_poses_topic", "goal_poses")),
      preview_goal_topic_(declare_parameter<std::string>("preview_goal_topic", "preview_goal")),
      preview_goals_topic_(declare_parameter<std::string>("preview_goals_topic", "preview_goals")),
      cancel_topic_(declare_parameter<std::string>("cancel_topic", "cancel_goal")),
      status_topic_(declare_parameter<std::string>("status_topic", "goal_status")) {
  listener_ = std::make_unique<TransformListener>(buffer_, *this);
  planner_ = std::make_unique<PlannerClient>(context, msgs::nav2::COMPUTE_PATH_TO_POSE_ACTION);
  controller_ = std::make_unique<ControllerClient>(context, msgs::nav2::FOLLOW_PATH_ACTION);
  self_client_ = std::make_unique<SelfClient>(context, msgs::nav2::NAVIGATE_TO_POSE_ACTION);
  spin_ = std::make_unique<SpinClient>(context, msgs::nav2::SPIN_ACTION);
  backup_ = std::make_unique<BackUpClient>(context, msgs::nav2::BACKUP_ACTION);
  wait_ = std::make_unique<WaitClient>(context, msgs::nav2::WAIT_ACTION);
  action_ = std::make_unique<ActionServer<msgs::nav2::NavigateToPoseGoal,
                                          msgs::nav2::NavigateToPoseFeedback,
                                          msgs::nav2::NavigateToPoseResult>>(
      context, msgs::nav2::NAVIGATE_TO_POSE_ACTION,
      [this](NavigateHandle& handle) { return execute(handle); },
      [this](const msgs::nav2::NavigateToPoseGoal& goal) {
        (void)goal;
        preempt_.store(true);
        return true;
      });
  pose_subscriptions_.push_back(subscribe<msgs::PoseStamped>(
      goal_topic_.value(), [this](const msgs::PoseStamped& pose) { on_goal_pose(pose); }));
  goal_poses_subscription_ = subscribe<msgs::Path>(
      goal_poses_topic_.value(), [this](const msgs::Path& goals) { on_goal_poses(goals); });
  pose_subscriptions_.push_back(subscribe<msgs::PoseStamped>(
      preview_goal_topic_.value(), [this](const msgs::PoseStamped& pose) { on_preview_goal(pose); }));
  preview_goals_subscription_ = subscribe<msgs::Path>(
      preview_goals_topic_.value(), [this](const msgs::Path& goals) { on_preview_goals(goals); });
  cancel_subscription_ = subscribe<msgs::Bool>(
      cancel_topic_.value(), [this](const msgs::Bool& message) { on_cancel(message); });
  logger().info("bt_navigator: goal=%s global=%s robot=%s", goal_topic_.value().c_str(),
                global_frame_.value().c_str(), robot_base_frame_.value().c_str());
}

BtNavigator::~BtNavigator() {
  cancel_requested_.store(true);
  preempt_.store(true);
  if (const auto follow = current_follow()) {
    follow->cancel();
  }
  if (action_) {
    action_->shutdown();
  }
  if (through_thread_.joinable()) {
    through_thread_.join();
  }
}

std::shared_ptr<BtNavigator::FollowHandle> BtNavigator::current_follow() const {
  std::lock_guard<std::mutex> lock(follow_mutex_);
  return follow_handle_;
}

void BtNavigator::set_follow(std::shared_ptr<FollowHandle> follow) {
  std::lock_guard<std::mutex> lock(follow_mutex_);
  follow_handle_ = std::move(follow);
}

std::optional<msgs::PoseStamped> BtNavigator::robot_pose() const {
  try {
    const msgs::TransformStamped transform =
        buffer_.lookup_transform(global_frame_.value(), robot_base_frame_.value());
    msgs::PoseStamped pose;
    pose.header = msgs::Header{now(), global_frame_.value()};
    pose.pose.position = {transform.transform.translation.x, transform.transform.translation.y, 0.0};
    pose.pose.orientation = transform.transform.rotation;
    return pose;
  } catch (const TransformException&) {
    return std::nullopt;
  }
}

void BtNavigator::publish_status(const std::string& text) {
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (text == last_status_) {
      return;
    }
    last_status_ = text;
  }
  publish(status_topic_.value(), msgs::String{text}, QOS_LATCHED);
  logger().info("bt_navigator: %s", text.c_str());
}

void BtNavigator::publish_empty_plan() {
  msgs::Path path;
  path.header = msgs::Header{now(), global_frame_.value()};
  publish("plan", std::move(path), QOS_LATCHED);
}

void BtNavigator::on_goal_pose(const msgs::PoseStamped& pose) {
  logger().info("goal_pose: (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y);
  cancel_requested_.store(true);
  if (const auto follow = current_follow()) {
    follow->cancel();
  }
  cancel_requested_.store(false);
  try {
    self_client_->send_goal(msgs::nav2::NavigateToPoseGoal{pose, ""});
  } catch (const std::exception& error) {
    logger().warn("navigate_to_pose goal을 보내지 못했다: %s", error.what());
  }
}

void BtNavigator::on_preview_goal(const msgs::PoseStamped& pose) {
  logger().info("preview_goal: (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y);
  const auto plan = plan_goal(pose);
  if (plan.error_code != msgs::nav2::ComputePathToPoseResult::NONE) {
    logger().warn("preview: 계획 실패 (%d): %s", plan.error_code, plan.error_msg.c_str());
    return;
  }
  publish("plan", plan.path, QOS_LATCHED);
  publish_status("preview");
}

void BtNavigator::on_preview_goals(const msgs::Path& goals) {
  if (goals.poses.empty()) {
    return;
  }
  logger().info("preview_goals: %zu goals", goals.poses.size());
  const auto combined = plan_through(goals.poses);
  if (!combined) {
    return;
  }
  publish("plan", *combined, QOS_LATCHED);
  publish_status("preview");
}

void BtNavigator::on_goal_poses(const msgs::Path& goals) {
  if (goals.poses.empty()) {
    return;
  }
  logger().info("goal_poses: %zu goals", goals.poses.size());
  cancel_requested_.store(true);
  if (const auto follow = current_follow()) {
    follow->cancel();
  }
  if (through_thread_.joinable()) {
    through_thread_.join();
  }
  cancel_requested_.store(false);
  through_thread_ = std::thread([this, poses = goals.poses] { run_through(poses); });
}

void BtNavigator::on_cancel(const msgs::Bool& message) {
  (void)message;
  logger().info("cancel requested");
  cancel_requested_.store(true);
  preempt_.store(true);
  if (const auto follow = current_follow()) {
    follow->cancel();
  }
  publish_empty_plan();
  publish_status("canceled");
}

msgs::nav2::ComputePathToPoseResult BtNavigator::plan_segment(
    const msgs::PoseStamped& start, const msgs::PoseStamped& goal) {
  msgs::nav2::ComputePathToPoseResult result;
  try {
    msgs::nav2::ComputePathToPoseGoal request;
    request.goal = goal;
    request.start = start;
    request.use_start = true;
    const auto handle = planner_->send_goal(request);
    const ActionResult action_result = handle->result(30.0);
    if (action_result.status != GoalStatus::Succeeded || !action_result.result.has_value()) {
      result.error_code = msgs::nav2::ComputePathToPoseResult::UNKNOWN;
      result.error_msg = "planner_server 결과가 없다 (" + status_name(action_result.status) + ")";
      return result;
    }
    return std::any_cast<msgs::nav2::ComputePathToPoseResult>(action_result.result);
  } catch (const std::exception& error) {
    result.error_code = msgs::nav2::ComputePathToPoseResult::UNKNOWN;
    result.error_msg = error.what();
    return result;
  }
}

msgs::nav2::ComputePathToPoseResult BtNavigator::plan_goal(const msgs::PoseStamped& goal) {
  msgs::nav2::ComputePathToPoseResult result;
  try {
    msgs::nav2::ComputePathToPoseGoal request;
    request.goal = goal;
    const auto handle = planner_->send_goal(request);
    const ActionResult action_result = handle->result(30.0);
    if (action_result.status != GoalStatus::Succeeded || !action_result.result.has_value()) {
      result.error_code = msgs::nav2::ComputePathToPoseResult::UNKNOWN;
      result.error_msg = "planner_server 결과가 없다 (" + status_name(action_result.status) + ")";
      return result;
    }
    return std::any_cast<msgs::nav2::ComputePathToPoseResult>(action_result.result);
  } catch (const std::exception& error) {
    result.error_code = msgs::nav2::ComputePathToPoseResult::UNKNOWN;
    result.error_msg = error.what();
    return result;
  }
}

std::optional<msgs::Path> BtNavigator::plan_through(
    const std::vector<msgs::PoseStamped>& goals) {
  const auto start = robot_pose();
  if (!start) {
    logger().warn("goal_poses: 로봇 위치를 모른다");
    return std::nullopt;
  }
  msgs::PoseStamped current = *start;
  std::vector<msgs::PoseStamped> combined;
  for (const auto& goal : goals) {
    if (cancel_requested_.load()) {
      return std::nullopt;
    }
    const auto plan = plan_segment(current, goal);
    if (plan.error_code != msgs::nav2::ComputePathToPoseResult::NONE) {
      logger().warn("goal_poses: 계획 실패 (%d): %s", plan.error_code, plan.error_msg.c_str());
      return std::nullopt;
    }
    std::vector<msgs::PoseStamped> poses = plan.path.poses;
    if (!combined.empty() && !poses.empty()) {
      const auto& last = combined.back().pose.position;
      const auto& first = poses.front().pose.position;
      if (last.x == first.x && last.y == first.y) {
        poses.erase(poses.begin());
      }
    }
    combined.insert(combined.end(), poses.begin(), poses.end());
    current = goal;
  }
  if (combined.empty() || cancel_requested_.load()) {
    return std::nullopt;
  }
  msgs::Path path;
  path.header = msgs::Header{now(), global_frame_.value()};
  path.poses = std::move(combined);
  return path;
}

void BtNavigator::run_through(std::vector<msgs::PoseStamped> goals) {
  cancel_requested_.store(false);
  const auto combined = plan_through(goals);
  if (!combined || cancel_requested_.load()) {
    return;
  }
  logger().info("goal_poses: combined path %zu poses", combined->poses.size());
  publish("plan", *combined, QOS_LATCHED);
  std::shared_ptr<FollowHandle> follow;
  try {
    msgs::nav2::FollowPathGoal request;
    request.path = *combined;
    follow = controller_->send_goal(request);
  } catch (const std::exception& error) {
    logger().warn("goal_poses: controller goal을 보내지 못했다: %s", error.what());
    return;
  }
  set_follow(follow);
  while (follow->status() == GoalStatus::Accepted && !cancel_requested_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (cancel_requested_.load()) {
    follow->cancel();
    publish_status("canceled");
  } else {
    publish_status("reached");
  }
  publish_empty_plan();
  set_follow(nullptr);
}

void BtNavigator::recover() {
  publish_status("recovering");
  logger().info("recovery: spin");
  if (cancel_requested_.load()) {
    return;
  }
  try {
    msgs::nav2::SpinGoal goal;
    goal.target_yaw = kPi;
    goal.time_allowance = 5.0;
    spin_->send_goal(goal)->result(10.0);
  } catch (const std::exception& error) {
    logger().warn("recovery spin failed: %s", error.what());
  }
  if (cancel_requested_.load()) {
    return;
  }
  if (!recovery_backup_.value()) {
    logger().info("recovery: backup (disabled)");
  } else {
    logger().info("recovery: backup");
    try {
      msgs::nav2::BackUpGoal goal;
      goal.backup_dist = 0.5;
      goal.backup_speed = 0.1;
      goal.time_allowance = 5.0;
      backup_->send_goal(goal)->result(10.0);
    } catch (const std::exception& error) {
      logger().warn("recovery backup failed: %s", error.what());
    }
  }
  if (cancel_requested_.load()) {
    return;
  }
  logger().info("recovery: wait");
  try {
    msgs::nav2::WaitGoal goal;
    goal.wait_duration = 1.0;
    wait_->send_goal(goal)->result(5.0);
  } catch (const std::exception& error) {
    logger().warn("recovery wait failed: %s", error.what());
  }
}

msgs::nav2::NavigateToPoseFeedback BtNavigator::feedback(const msgs::PoseStamped& goal,
                                                         double began, int recoveries) const {
  msgs::nav2::NavigateToPoseFeedback message;
  message.navigation_time = now() - began;
  message.number_of_recoveries = recoveries;
  if (const auto pose = robot_pose()) {
    message.current_pose = *pose;
    message.distance_remaining =
        std::hypot(goal.pose.position.x - pose->pose.position.x,
                   goal.pose.position.y - pose->pose.position.y);
  }
  return message;
}

msgs::nav2::NavigateToPoseResult BtNavigator::execute(NavigateHandle& handle) {
  msgs::nav2::NavigateToPoseResult result;
  preempt_.store(false);
  cancel_requested_.store(false);
  const msgs::PoseStamped goal = handle.goal().pose;
  const double began = now();
  int recoveries = 0;
  auto plan = plan_goal(goal);
  if (plan.error_code != msgs::nav2::ComputePathToPoseResult::NONE) {
    publish_status("plan failed");
    result.error_code = plan.error_code;
    result.error_msg = plan.error_msg;
    logger().warn("내비게이션 실패 (%d): %s", plan.error_code, plan.error_msg.c_str());
    return result;
  }
  std::shared_ptr<FollowHandle> follow;
  try {
    msgs::nav2::FollowPathGoal request;
    request.path = plan.path;
    follow = controller_->send_goal(request);
  } catch (const std::exception& error) {
    result.error_code = msgs::nav2::NavigateToPoseResult::UNKNOWN;
    result.error_msg = error.what();
    return result;
  }
  set_follow(follow);
  publish_status("following");
  double planned_at = now();
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (cancel_requested_.load() || handle.is_cancel_requested()) {
      follow->cancel();
      set_follow(nullptr);
      publish_status("canceled");
      result.error_code = msgs::nav2::NavigateToPoseResult::NONE;
      result.error_msg = "취소";
      return result;
    }
    if (preempt_.load()) {
      set_follow(nullptr);
      result.error_code = msgs::nav2::NavigateToPoseResult::NONE;
      result.error_msg = kReplaced;
      return result;
    }
    if (follow->status() != GoalStatus::Accepted) {
      const ActionResult action_result = follow->result(5.0);
      int code = msgs::nav2::FollowPathResult::UNKNOWN;
      std::string message = "controller_server 결과가 없다 (" + status_name(action_result.status) + ")";
      if (action_result.result.has_value()) {
        try {
          const auto follow_result =
              std::any_cast<msgs::nav2::FollowPathResult>(action_result.result);
          code = follow_result.error_code;
          message = follow_result.error_msg;
        } catch (const std::bad_any_cast&) {
        }
      }
      if (code == msgs::nav2::FollowPathResult::NONE && message != kReplaced) {
        publish_empty_plan();
        publish_status("reached");
        logger().info("goal reached: (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
        result.error_code = msgs::nav2::NavigateToPoseResult::NONE;
        return result;
      }
      if (code == msgs::nav2::FollowPathResult::NONE) {
        continue;
      }
      if (recoveries >= number_of_recoveries_.value()) {
        publish_empty_plan();
        publish_status("failed");
        result.error_code = code;
        result.error_msg = message;
        logger().warn("내비게이션 실패 (%d): %s", code, message.c_str());
        return result;
      }
      ++recoveries;
      recover();
      if (cancel_requested_.load()) {
        result.error_code = msgs::nav2::NavigateToPoseResult::NONE;
        result.error_msg = "취소";
        return result;
      }
      logger().warn("따라가기 실패 (%d: %s): 다시 계획한다 (%d/%lld)", code, message.c_str(),
                    recoveries, static_cast<long long>(number_of_recoveries_.value()));
      planned_at = -std::numeric_limits<double>::infinity();
    }
    if (now() - planned_at >= replanning_period_.value()) {
      logger().info("replanning (period %.1fs)", replanning_period_.value());
      plan = plan_goal(goal);
      planned_at = now();
      if (plan.error_code != msgs::nav2::ComputePathToPoseResult::NONE) {
        if (follow->status() != GoalStatus::Accepted) {
          result.error_code = plan.error_code;
          result.error_msg = plan.error_msg;
          logger().warn("내비게이션 실패 (%d): %s", plan.error_code, plan.error_msg.c_str());
          return result;
        }
      } else {
        try {
          msgs::nav2::FollowPathGoal request;
          request.path = plan.path;
          follow = controller_->send_goal(request);
          set_follow(follow);
        } catch (const std::exception& error) {
          logger().warn("재계획 경로를 보내지 못했다: %s", error.what());
        }
      }
    }
    handle.publish_feedback(feedback(goal, began, recoveries));
  }
}

}  // namespace krosc::nodes::nav2
