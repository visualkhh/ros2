#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/ros2_control/controller_interface.hpp"
#include "krosc/tf_ros.hpp"
#include "krosc/topic.hpp"

namespace krosc::ros2_control {

// ros2_control diff_drive_controller 흉내.
//   cmd_vel -> 바퀴 velocity command, 바퀴 position state -> odom(+선택 TF).
//   실제 ROS2처럼 바퀴 명령은 hardware interface를 통해 나간다.
class DiffDriveController final : public ControllerInterface {
 public:
  bool on_init(ControllerContext& context) override;
  bool on_activate() override;
  bool on_deactivate() override;
  void update(double time, double period) override;

  [[nodiscard]] std::vector<std::string> command_interface_names() const override;
  [[nodiscard]] std::string type() const override {
    return "diff_drive_controller/DiffDriveController";
  }

 private:
  void on_cmd_vel(const msgs::Twist& message);
  void publish_odometry(double time, double linear, double angular);

  Parameter<std::string> left_wheel_;
  Parameter<std::string> right_wheel_;
  Parameter<std::string> cmd_vel_topic_;
  Parameter<std::string> odom_topic_;
  Parameter<std::string> odom_frame_;
  Parameter<std::string> base_frame_;
  Parameter<double> wheel_separation_;
  Parameter<double> wheel_radius_;
  Parameter<double> command_timeout_;
  Parameter<double> jump_threshold_;
  Parameter<double> odom_publish_rate_;
  Parameter<bool> publish_tf_;
  Parameter<std::vector<double>> pose_covariance_;
  Parameter<std::vector<double>> twist_covariance_;

  std::shared_ptr<StateInterface> left_state_;
  std::shared_ptr<StateInterface> right_state_;
  std::shared_ptr<CommandInterface> left_command_;
  std::shared_ptr<CommandInterface> right_command_;
  Subscription<msgs::Twist> command_subscription_;
  std::unique_ptr<TransformBroadcaster> broadcaster_;

  std::mutex command_mutex_;
  msgs::Twist command_;
  double command_time_ = -1e18;

  bool has_previous_ = false;
  double previous_left_ = 0.0;
  double previous_right_ = 0.0;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double last_publish_ = -1e18;
};

}  // namespace krosc::ros2_control
