#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "krosc/action.hpp"
#include "krosc/msgs/control_msgs.hpp"
#include "krosc/ros2_control/controller_interface.hpp"

namespace krosc::ros2_control {

// ros2_control joint_trajectory_controller 흉내.
//   FollowJointTrajectory 액션으로 궤적을 받아 update()에서 위치 command interface로 보간해 쓴다.
class JointTrajectoryController final : public ControllerInterface {
 public:
  ~JointTrajectoryController() override;

  bool on_init(ControllerContext& context) override;
  bool on_configure() override;
  bool on_activate() override;
  bool on_deactivate() override;
  bool on_cleanup() override;

  void update(double time, double period) override;
  [[nodiscard]] std::vector<std::string> command_interface_names() const override;
  [[nodiscard]] std::string type() const override {
    return "joint_trajectory_controller/JointTrajectoryController";
  }

 private:
  using Handle = ServerGoalHandle<msgs::control::FollowJointTrajectoryGoal,
                                  msgs::control::FollowJointTrajectoryFeedback>;

  struct Trajectory {
    std::vector<double> times;
    std::vector<std::vector<double>> positions;  // [joint][point]
    double duration = 0.0;
  };

  [[nodiscard]] bool set_trajectory(const msgs::control::FollowJointTrajectoryGoal& goal,
                                    std::string& error);
  [[nodiscard]] std::vector<double> sample(double elapsed) const;
  void apply_hold();
  void publish_state(double time, const std::vector<double>& desired,
                     const std::vector<double>& actual, bool force);
  msgs::control::FollowJointTrajectoryResult execute(Handle& handle);

  std::vector<std::string> joints_;
  std::vector<std::shared_ptr<CommandInterface>> commands_;
  std::vector<std::shared_ptr<StateInterface>> states_;
  Parameter<std::string> action_name_;
  Parameter<std::string> state_topic_;
  Parameter<double> goal_time_tolerance_;

  mutable std::mutex mutex_;
  Trajectory trajectory_;
  double start_time_ = 0.0;
  std::vector<double> actual_;
  std::vector<double> desired_;
  std::atomic<bool> active_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> done_{false};
  std::atomic<double> progress_{0.0};
  double last_state_time_ = -1e18;

  std::unique_ptr<ActionServer<msgs::control::FollowJointTrajectoryGoal,
                               msgs::control::FollowJointTrajectoryFeedback,
                               msgs::control::FollowJointTrajectoryResult>>
      action_;
};

}  // namespace krosc::ros2_control
