#include "krosc/control/diff_drive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "krosc/tf/math.hpp"
#include "krosc/qos.hpp"

namespace krosc::control {

namespace {

constexpr double kStoppedVariance = 1e-6;

std::array<double, 36> covariance_matrix(const std::vector<double>& diagonal) {
  std::array<double, 36> matrix{};
  for (std::size_t i = 0; i < 6 && i < diagonal.size(); ++i) {
    matrix[i * 6 + i] = diagonal[i];
  }
  return matrix;
}

bool valid_diagonal(const std::vector<double>& value) {
  if (value.size() != 6) {
    return false;
  }
  for (double item : value) {
    if (item < 0.0) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool DiffDriveController::on_init(ControllerContext& context) {
  if (!Controller::on_init(context)) {
    return false;
  }
  left_wheel_ = context.node->declare_parameter<std::string>(context.name + ".left_wheel",
                                                             "wheel_left_joint");
  right_wheel_ = context.node->declare_parameter<std::string>(context.name + ".right_wheel",
                                                              "wheel_right_joint");
  cmd_vel_topic_ = context.node->declare_parameter<std::string>(context.name + ".cmd_vel_topic",
                                                                "cmd_vel");
  odom_topic_ = context.node->declare_parameter<std::string>(context.name + ".odom_topic", "odom");
  odom_frame_ = context.node->declare_parameter<std::string>(context.name + ".odom_frame", "odom");
  base_frame_ = context.node->declare_parameter<std::string>(context.name + ".base_frame",
                                                             "base_footprint");
  wheel_separation_ = context.node->declare_parameter<double>(
      context.name + ".wheel_separation", 0.288, [](double value) { return value > 0.0; });
  wheel_radius_ = context.node->declare_parameter<double>(
      context.name + ".wheel_radius", 0.033, [](double value) { return value > 0.0; });
  command_timeout_ = context.node->declare_parameter<double>(
      context.name + ".command_timeout", 0.5, [](double value) { return value > 0.0; });
  jump_threshold_ = context.node->declare_parameter<double>(
      context.name + ".jump_threshold", 1.0, [](double value) { return value > 0.0; });
  odom_publish_rate_ = context.node->declare_parameter<double>(
      context.name + ".odom_publish_rate", 20.0, [](double value) { return value > 0.0; });
  publish_tf_ = context.node->declare_parameter<bool>(context.name + ".publish_tf", false);
  pose_covariance_ = context.node->declare_parameter<std::vector<double>>(
      context.name + ".pose_covariance_diagonal",
      std::vector<double>{0.001, 0.001, 1e6, 1e6, 1e6, 0.01}, valid_diagonal);
  twist_covariance_ = context.node->declare_parameter<std::vector<double>>(
      context.name + ".twist_covariance_diagonal",
      std::vector<double>{0.001, 0.001, 1e6, 1e6, 1e6, 0.01}, valid_diagonal);

  left_state_ = context.state(left_wheel_.value() + "/position");
  right_state_ = context.state(right_wheel_.value() + "/position");
  left_command_ = context.command(left_wheel_.value() + "/velocity");
  right_command_ = context.command(right_wheel_.value() + "/velocity");
  if (!left_state_ || !right_state_ || !left_command_ || !right_command_) {
    context.node->logger().error(
        "diff_drive_controller: 바퀴 state/velocity command 인터페이스가 없다 (%s, %s)",
        left_wheel_.value().c_str(), right_wheel_.value().c_str());
    return false;
  }
  command_subscription_ = context.node->subscribe<msgs::Twist>(
      cmd_vel_topic_.value(), [this](const msgs::Twist& message) { on_cmd_vel(message); },
      QOS_COMMAND);
  if (publish_tf_.value()) {
    broadcaster_ = std::make_unique<TransformBroadcaster>(*context.node);
  }
  return true;
}

bool DiffDriveController::on_activate() {
  previous_left_ = left_state_->get();
  previous_right_ = right_state_->get();
  has_previous_ = true;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = msgs::Twist{};
    command_time_ = -1e18;
  }
  last_publish_ = -1e18;
  return true;
}

bool DiffDriveController::on_deactivate() {
  if (left_command_ && right_command_) {
    left_command_->set(0.0);
    right_command_->set(0.0);
  }
  return true;
}

void DiffDriveController::on_cmd_vel(const msgs::Twist& message) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_ = message;
  command_time_ = context().node->now();
}

std::vector<std::string> DiffDriveController::command_interface_names() const {
  return {left_wheel_.value() + "/velocity", right_wheel_.value() + "/velocity"};
}

void DiffDriveController::update(double time, double period) {
  const double left = left_state_->get();
  const double right = right_state_->get();
  double linear = 0.0;
  double angular = 0.0;
  if (!has_previous_) {
    previous_left_ = left;
    previous_right_ = right;
    has_previous_ = true;
  } else {
    const double delta_left = left - previous_left_;
    const double delta_right = right - previous_right_;
    if (std::abs(delta_left) <= jump_threshold_.value() &&
        std::abs(delta_right) <= jump_threshold_.value() && period > 1e-9) {
      const double radius = wheel_radius_.value();
      const double separation = wheel_separation_.value();
      const double distance = radius * (delta_left + delta_right) / 2.0;
      const double turn = radius * (delta_right - delta_left) / separation;
      x_ += distance * std::cos(yaw_ + turn / 2.0);
      y_ += distance * std::sin(yaw_ + turn / 2.0);
      yaw_ = std::atan2(std::sin(yaw_ + turn), std::cos(yaw_ + turn));
      linear = distance / period;
      angular = turn / period;
    }
    previous_left_ = left;
    previous_right_ = right;
  }
  if (time - last_publish_ >= 1.0 / odom_publish_rate_.value()) {
    last_publish_ = time;
    publish_odometry(time, linear, angular);
  }

  msgs::Twist command;
  bool fresh = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command = command_;
    fresh = time - command_time_ <= command_timeout_.value();
  }
  if (!fresh) {
    command = msgs::Twist{};
  }
  const double half = wheel_separation_.value() / 2.0;
  const double radius = wheel_radius_.value();
  left_command_->set((command.linear.x - command.angular.z * half) / radius);
  right_command_->set((command.linear.x + command.angular.z * half) / radius);
}

void DiffDriveController::publish_odometry(double time, double linear, double angular) {
  msgs::Odometry odometry;
  odometry.header.stamp = time;
  odometry.header.frame_id = odom_frame_.value();
  odometry.child_frame_id = base_frame_.value();
  odometry.pose.pose.position = {x_, y_, 0.0};
  odometry.pose.pose.orientation = tfmath::from_yaw(yaw_);
  odometry.pose.covariance = covariance_matrix(pose_covariance_.value());
  std::vector<double> twist = twist_covariance_.value();
  if (linear == 0.0 && angular == 0.0 && twist.size() == 6) {
    twist[0] = kStoppedVariance;
    twist[1] = kStoppedVariance;
    twist[5] = kStoppedVariance;
  }
  odometry.twist.twist.linear.x = linear;
  odometry.twist.twist.angular.z = angular;
  odometry.twist.covariance = covariance_matrix(twist);
  context().node->publish(odom_topic_.value(), std::move(odometry), QOS_DEFAULT);

  if (broadcaster_) {
    msgs::TransformStamped transform;
    transform.header.stamp = time;
    transform.header.frame_id = odom_frame_.value();
    transform.child_frame_id = base_frame_.value();
    transform.transform.translation = {x_, y_, 0.0};
    transform.transform.rotation = tfmath::from_yaw(yaw_);
    broadcaster_->send_transform(transform);
  }
}

}  // namespace krosc::control
