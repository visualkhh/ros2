#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "krosc/action.hpp"
#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/tf/tf.hpp"
#include "krosc/tf/tf_ros.hpp"

namespace krosc::nodes::nav2 {

// nav2 bt_navigator 흉내: navigate_to_pose / navigate_through_poses 액션을 받아
// planner_server로 계획하고 controller_server로 따라가게 한다. 두 액션 모두 nav2 표준 인터페이스다.
// nav2 기본 BT(navigate_to_pose_w_replanning_and_recovery)의 흐름을 코드로 고정했다:
// replanning_period마다 다시 계획해 새 경로를 보내고, controller가 실패하면 number_of_recoveries번까지
// behavior_server(spin → backup → wait)로 회복한 뒤 다시 계획한다. 새 goal이 오면 지금 goal은 물러난다.
class BtNavigator final : public Node {
 public:
  explicit BtNavigator(Context& context, std::string name = "bt_navigator",
                       std::map<std::string, ParamValue> parameters = {});
  ~BtNavigator() override;

  BtNavigator(const BtNavigator&) = delete;
  BtNavigator& operator=(const BtNavigator&) = delete;

 private:
  using NavigateHandle = ServerGoalHandle<msgs::nav2::NavigateToPoseGoal,
                                          msgs::nav2::NavigateToPoseFeedback>;
  using ThroughHandle = ServerGoalHandle<msgs::nav2::NavigateThroughPosesGoal,
                                         msgs::nav2::NavigateThroughPosesFeedback>;
  using PlannerClient =
      ActionClient<msgs::nav2::ComputePathToPoseGoal, msgs::nav2::ComputePathToPoseFeedback,
                   msgs::nav2::ComputePathToPoseResult>;
  using ControllerClient = ActionClient<msgs::nav2::FollowPathGoal, msgs::nav2::FollowPathFeedback,
                                        msgs::nav2::FollowPathResult>;
  using SpinClient =
      ActionClient<msgs::nav2::SpinGoal, msgs::nav2::SpinFeedback, msgs::nav2::SpinResult>;
  using BackUpClient =
      ActionClient<msgs::nav2::BackUpGoal, msgs::nav2::BackUpFeedback, msgs::nav2::BackUpResult>;
  using WaitClient =
      ActionClient<msgs::nav2::WaitGoal, msgs::nav2::WaitFeedback, msgs::nav2::WaitResult>;
  using FollowHandle = ClientGoalHandle<msgs::nav2::FollowPathGoal,
                                        msgs::nav2::FollowPathFeedback,
                                        msgs::nav2::FollowPathResult>;

  [[nodiscard]] msgs::nav2::ComputePathToPoseResult plan_segment(const msgs::PoseStamped& start,
                                                                 const msgs::PoseStamped& goal);
  [[nodiscard]] msgs::nav2::ComputePathToPoseResult plan_goal(const msgs::PoseStamped& goal);
  [[nodiscard]] std::optional<msgs::Path> plan_through(const std::vector<msgs::PoseStamped>& goals);
  [[nodiscard]] msgs::nav2::NavigateToPoseResult execute(NavigateHandle& handle);
  [[nodiscard]] msgs::nav2::NavigateThroughPosesResult execute_through(ThroughHandle& handle);
  void recover();
  void publish_status(const std::string& text);
  void publish_empty_plan();
  [[nodiscard]] std::optional<msgs::PoseStamped> robot_pose() const;
  [[nodiscard]] msgs::nav2::NavigateToPoseFeedback feedback(const msgs::PoseStamped& goal,
                                                            double began,
                                                            int recoveries) const;
  [[nodiscard]] msgs::nav2::NavigateThroughPosesFeedback through_feedback(
      const msgs::PoseStamped& goal, double began, int recoveries, std::uint32_t waypoint,
      const msgs::PoseStamped& current,
      const std::optional<msgs::PoseStamped>& robot) const;
  [[nodiscard]] std::shared_ptr<FollowHandle> current_follow() const;
  void set_follow(std::shared_ptr<FollowHandle> follow);

  Parameter<double> replanning_period_;
  Parameter<std::int64_t> number_of_recoveries_;
  Parameter<bool> recovery_backup_;
  Parameter<std::string> global_frame_;
  Parameter<std::string> robot_base_frame_;
  Parameter<std::string> status_topic_;

  Buffer buffer_;
  std::unique_ptr<TransformListener> listener_;
  std::unique_ptr<PlannerClient> planner_;
  std::unique_ptr<ControllerClient> controller_;
  std::unique_ptr<SpinClient> spin_;
  std::unique_ptr<BackUpClient> backup_;
  std::unique_ptr<WaitClient> wait_;
  std::unique_ptr<ActionServer<msgs::nav2::NavigateToPoseGoal, msgs::nav2::NavigateToPoseFeedback,
                               msgs::nav2::NavigateToPoseResult>>
      action_;
  std::unique_ptr<ActionServer<msgs::nav2::NavigateThroughPosesGoal,
                               msgs::nav2::NavigateThroughPosesFeedback,
                               msgs::nav2::NavigateThroughPosesResult>>
      through_action_;

  std::atomic<bool> preempt_{false};
  std::atomic<bool> cancel_requested_{false};
  mutable std::mutex follow_mutex_;
  std::shared_ptr<FollowHandle> follow_handle_;
  mutable std::mutex status_mutex_;
  std::string last_status_;
};

}  // namespace krosc::nodes::nav2
