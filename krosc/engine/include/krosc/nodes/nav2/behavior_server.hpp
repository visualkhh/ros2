#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "krosc/action.hpp"
#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/tf/tf.hpp"

namespace krosc::nodes::nav2 {

// nav2 behavior_server 흉내: spin(제자리 회전) · backup(후진) · wait(대기).
// cmd_vel 토픽에 직접 속도를 내고 odom으로 진행을 확인한다. bt_navigator가 controller 실패 시 부른다.
class BehaviorServer final : public Node {
 public:
  explicit BehaviorServer(Context& context, std::string name = "behavior_server",
                          std::map<std::string, ParamValue> parameters = {});
  ~BehaviorServer() override;

  BehaviorServer(const BehaviorServer&) = delete;
  BehaviorServer& operator=(const BehaviorServer&) = delete;

 private:
  using SpinHandle = ServerGoalHandle<msgs::nav2::SpinGoal, msgs::nav2::SpinFeedback>;
  using BackUpHandle = ServerGoalHandle<msgs::nav2::BackUpGoal, msgs::nav2::BackUpFeedback>;
  using WaitHandle = ServerGoalHandle<msgs::nav2::WaitGoal, msgs::nav2::WaitFeedback>;

  msgs::nav2::SpinResult execute_spin(SpinHandle& handle);
  msgs::nav2::BackUpResult execute_backup(BackUpHandle& handle);
  msgs::nav2::WaitResult execute_wait(WaitHandle& handle);
  void on_odom(const msgs::Odometry& odom);
  void publish_velocity(double linear, double angular);
  void stop();

  Parameter<std::string> cmd_vel_topic_;
  Parameter<std::string> odom_topic_;
  Parameter<double> max_ang_;
  Parameter<double> max_lin_;

  mutable std::mutex odom_mutex_;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;
  bool has_odom_ = false;

  Subscription<msgs::Odometry> odom_subscription_;
  std::unique_ptr<ActionServer<msgs::nav2::SpinGoal, msgs::nav2::SpinFeedback, msgs::nav2::SpinResult>>
      spin_action_;
  std::unique_ptr<ActionServer<msgs::nav2::BackUpGoal, msgs::nav2::BackUpFeedback,
                               msgs::nav2::BackUpResult>>
      backup_action_;
  std::unique_ptr<ActionServer<msgs::nav2::WaitGoal, msgs::nav2::WaitFeedback, msgs::nav2::WaitResult>>
      wait_action_;
};

}  // namespace krosc::nodes::nav2
