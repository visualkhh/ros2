#pragma once

#include <array>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "krosc/action.hpp"
#include "krosc/manipulation/arm_chain.hpp"
#include "krosc/moveit/controllers/controller_manager.hpp"
#include "krosc/moveit/kinematics/kinematics_base.hpp"
#include "krosc/moveit/planners/planner_manager.hpp"
#include "krosc/msgs/control_msgs.hpp"
#include "krosc/msgs/moveit_msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/timer.hpp"

namespace krosc::moveit {

// MoveIt move_group의 축소판.
//   move_action(nav2식 action) + 기존 Viz용 moveit_* 토픽을 함께 제공한다.
//   무거운 부분은 전부 플러그인으로 분리한다: kinematics / planner / controller_manager.
class MoveGroup final : public Node {
 public:
  using MoveHandle = ServerGoalHandle<msgs::moveit::MoveGroupGoal, msgs::moveit::MoveGroupFeedback>;

  explicit MoveGroup(Context& context, std::string name = "move_group",
                     std::map<std::string, ParamValue> parameters = {},
                     KinematicsRegistry kinematics = builtin_kinematics(),
                     PlannerRegistry planners = builtin_planners(),
                     ControllerManagerRegistry controllers = builtin_controller_managers());
  ~MoveGroup() override;

  MoveGroup(const MoveGroup&) = delete;
  MoveGroup& operator=(const MoveGroup&) = delete;

 private:
  struct Target {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double pitch = 0.0;
    double grip = 0.0;
    bool cartesian = true;
    bool preview = false;
  };

  void on_robot_description(const msgs::String& message);
  void on_joint_states(const msgs::JointState& message);
  void on_arm_status(const msgs::String& message);
  void on_target(const msgs::String& message);
  void on_plan(const msgs::Empty& message);
  void on_execute(const msgs::Empty& message);
  void on_cancel(const msgs::Empty& message);
  void on_scrub(const msgs::String& message);
  void tick();
  void heartbeat();

  [[nodiscard]] bool rebuild_chain(std::string& error);
  [[nodiscard]] bool current_joints(std::array<double, 4>& joints) const;
  [[nodiscard]] bool parse_target(const std::string& text, Target& target, std::string& error) const;
  [[nodiscard]] bool has_arm_error_locked() const;
  [[nodiscard]] std::array<int, 5> arm_errors_locked() const;
  [[nodiscard]] bool solve_ik_locked(const Target& target, const std::array<double, 4>& seed,
                                     std::array<double, 4>& goal, std::string& error,
                                     bool fast) const;
  [[nodiscard]] std::optional<manipulation::Trajectory> plan_trajectory_locked(
      const std::array<double, 4>& start, const std::array<double, 4>& goal, bool cartesian,
      std::string& error) const;
  [[nodiscard]] bool plan_locked(std::string& error);
  [[nodiscard]] bool goal_from_request_locked(const msgs::moveit::MoveGroupGoal& request,
                                              std::array<double, 4>& goal, std::string& error) const;
  [[nodiscard]] msgs::control::JointTrajectory to_message_locked(
      const manipulation::Trajectory& trajectory) const;
  void publish_status_locked(const std::string& text);
  void publish_state_locked();
  void publish_preview_locked(int index);
  void publish_target_pose_locked();
  void publish_path_locked(const std::string& topic,
                           const std::vector<std::pair<std::string, msgs::Transform>>& links);
  void stop_execution_locked(const std::string& reason, bool publish_stop);
  [[nodiscard]] msgs::moveit::MoveGroupResult execute_move(
      const msgs::moveit::MoveGroupGoal& request, MoveHandle* handle);

  Parameter<std::string> base_link_;
  Parameter<std::string> tip_link_;
  Parameter<std::vector<std::string>> joint_names_;
  Parameter<std::vector<double>> lower_limits_;
  Parameter<std::vector<double>> upper_limits_;
  Parameter<double> max_joint_speed_;
  Parameter<double> max_joint_accel_;
  Parameter<double> cartesian_speed_;
  Parameter<double> cartesian_accel_;
  Parameter<double> sample_time_;
  Parameter<std::string> arm_stop_topic_;
  Parameter<std::string> kinematics_plugin_;
  Parameter<std::string> planner_plugin_;
  Parameter<std::string> controller_manager_plugin_;

  KinematicsRegistry kinematics_registry_;
  PlannerRegistry planner_registry_;
  ControllerManagerRegistry controller_registry_;
  std::unique_ptr<MoveItKinematicsPlugin> kinematics_;
  std::unique_ptr<MoveItPlannerPlugin> planner_;
  std::unique_ptr<MoveItControllerManagerPlugin> controller_manager_;

  mutable std::mutex mutex_;
  std::string robot_description_;
  manipulation::ArmChain chain_;
  bool chain_ready_ = false;
  std::optional<std::array<double, 4>> current_;
  double gripper_ = 0.0;
  std::optional<Target> target_;
  std::optional<std::array<double, 4>> goal_;
  std::optional<manipulation::Trajectory> trajectory_;
  std::vector<msgs::Vector3> preview_path_;
  std::array<int, 5> arm_errors_{{0, 0, 0, 0, 0}};
  bool executing_ = false;
  bool action_owned_ = false;
  double execution_start_ = 0.0;
  int preview_index_ = 0;
  double last_state_ = -1e18;
  double last_status_ = -1e18;
  double last_preview_ = -1e18;
  std::string status_text_ = "MoveIt: waiting for robot_description/joint_states";
  std::shared_ptr<FollowJointTrajectoryHandle> follow_;

  std::atomic<bool> busy_{false};
  std::atomic<bool> cancel_requested_{false};

  Subscription<msgs::String> robot_description_subscription_;
  Subscription<msgs::JointState> joint_states_subscription_;
  Subscription<msgs::String> arm_status_subscription_;
  Subscription<msgs::String> target_subscription_;
  Subscription<msgs::Empty> plan_subscription_;
  Subscription<msgs::Empty> execute_subscription_;
  Subscription<msgs::Empty> cancel_subscription_;
  Subscription<msgs::String> scrub_subscription_;
  std::unique_ptr<ActionServer<msgs::moveit::MoveGroupGoal, msgs::moveit::MoveGroupFeedback,
                               msgs::moveit::MoveGroupResult>>
      action_;
  Timer tick_timer_;
  Timer status_timer_;
};

}  // namespace krosc::moveit
