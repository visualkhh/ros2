#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "krosc/action.hpp"
#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/tf/tf.hpp"
#include "krosc/tf/tf_ros.hpp"
#include "krosc/time/timer.hpp"

#include "krosc/nodes/nav2/controllers/controller.hpp"
#include "krosc/nodes/nav2/costmap/costmap_2d_ros.hpp"

namespace krosc::nodes::nav2 {

// nav2 controller_server 흉내: follow_path 액션으로 경로를 받아 controller_frequency마다 cmd_vel을 낸다.
// local_costmap(Costmap2DROS)은 별도 노드에서 받는다. 경로가 새로 오면 진행 중인 goal은 물러난다.
class ControllerServer final : public Node {
 public:
  explicit ControllerServer(Context& context, Costmap2DROS& local_costmap,
                            std::string name = "controller_server",
                            std::map<std::string, ParamValue> parameters = {},
                            ControllerRegistry registry = builtin_controllers());
  ~ControllerServer() override;

  ControllerServer(const ControllerServer&) = delete;
  ControllerServer& operator=(const ControllerServer&) = delete;

  [[nodiscard]] msgs::nav2::FollowPathResult follow_path(const msgs::nav2::FollowPathGoal& goal);

 private:
  struct ProgressChecker {
    double required_movement_radius = 0.5;
    double movement_time_allowance = 10.0;
    double required_angle = 0.5;
    std::optional<std::pair<Pose2, double>> baseline;

    void reset() { baseline.reset(); }
    [[nodiscard]] bool check(const Pose2& pose, double now);
  };

  [[nodiscard]] msgs::nav2::FollowPathResult run(
      const msgs::nav2::FollowPathGoal& goal,
      ServerGoalHandle<msgs::nav2::FollowPathGoal, msgs::nav2::FollowPathFeedback>* handle);
  void on_odom(const msgs::Odometry& odom);
  void stop();
  void publish_command(double linear, double angular, const std::vector<Pose2>& local_plan,
                       double distance, const std::string& frame,
                       ServerGoalHandle<msgs::nav2::FollowPathGoal, msgs::nav2::FollowPathFeedback>* handle,
                       std::int64_t tick);
  [[nodiscard]] std::vector<Pose2> transform_path(const std::vector<Pose2>& path,
                                                  const std::string& source,
                                                  const std::string& target) const;
  [[nodiscard]] msgs::nav2::FollowPathResult fail(int code, const std::string& message);

  Costmap2DROS& costmap_;
  ControllerRegistry registry_;
  std::map<std::string, std::unique_ptr<NavControllerPlugin>> controllers_;
  Parameter<std::vector<std::string>> controller_plugins_;
  Parameter<double> controller_frequency_;
  Parameter<std::string> odom_topic_;
  Parameter<std::string> cmd_vel_topic_;
  Parameter<std::string> local_plan_topic_;
  Parameter<double> final_rotation_gain_;
  Parameter<double> final_rotation_speed_;
  Parameter<double> max_linear_accel_;
  Parameter<double> max_angular_accel_;
  Parameter<double> xy_goal_tolerance_;
  Parameter<double> yaw_goal_tolerance_;
  Parameter<double> startup_wait_;

  ProgressChecker progress_checker_;
  mutable std::mutex velocity_mutex_;
  Velocity velocity_;
  Velocity last_command_;
  std::atomic<bool> preempt_{false};
  std::mutex tick_mutex_;
  std::condition_variable tick_cv_;
  std::uint64_t tick_seq_ = 0;
  Subscription<msgs::Odometry> odom_subscription_;
  Timer tick_timer_;
  std::unique_ptr<ActionServer<msgs::nav2::FollowPathGoal, msgs::nav2::FollowPathFeedback,
                               msgs::nav2::FollowPathResult>>
      action_;
};

}  // namespace krosc::nodes::nav2
