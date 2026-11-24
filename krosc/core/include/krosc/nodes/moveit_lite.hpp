#pragma once

#include <array>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "krosc/manipulation/arm_chain.hpp"
#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/timer.hpp"
#include "krosc/topic.hpp"

namespace krosc::nodes {

// MoveIt-lite 코어 노드. URDF 체인으로 IK/궤적을 만들고 arm_joint_command로 실행한다.
// Viz는 moveit_target / moveit_plan / moveit_execute / moveit_cancel / moveit_scrub만 보내고,
// moveit_status / moveit_state / moveit_target_pose / moveit_arm / moveit_path를 받아 표시한다.
class MoveItLite final : public Node {
 public:
  explicit MoveItLite(Context& context, std::string name = "moveit_lite",
                      std::map<std::string, ParamValue> parameters = {});

 private:
  struct Target {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double pitch = 0.0;   // rad, 접근축 하향
    double grip = 0.0;    // m
    bool cartesian = true;
    bool preview = false;  // true면 IK/ghost만, 전체 궤적 계획은 하지 않는다
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

  // 아래 *_locked 함수는 mutex_를 잡은 상태에서만 호출한다.
  bool plan_locked(std::string& error);
  void publish_status_locked(const std::string& text);
  void publish_state_locked();
  void publish_preview_locked(int index);
  void publish_target_pose_locked();
  void publish_path_locked(const std::string& topic,
                           const std::vector<std::pair<std::string, msgs::Transform>>& links);
  void stop_execution_locked(const std::string& reason, bool publish_stop);

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
  Parameter<std::string> arm_command_topic_;
  Parameter<std::string> arm_stop_topic_;

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
  double execution_start_ = 0.0;
  int preview_index_ = 0;
  double last_state_ = -1e18;
  double last_status_ = -1e18;
  double last_preview_ = -1e18;
  std::string status_text_ = "MoveIt: waiting for robot_description/joint_states";

  Subscription<msgs::String> robot_description_subscription_;
  Subscription<msgs::JointState> joint_states_subscription_;
  Subscription<msgs::String> arm_status_subscription_;
  Subscription<msgs::String> target_subscription_;
  Subscription<msgs::Empty> plan_subscription_;
  Subscription<msgs::Empty> execute_subscription_;
  Subscription<msgs::Empty> cancel_subscription_;
  Subscription<msgs::String> scrub_subscription_;
  Timer tick_timer_;
  Timer status_timer_;
};

}  // namespace krosc::nodes
