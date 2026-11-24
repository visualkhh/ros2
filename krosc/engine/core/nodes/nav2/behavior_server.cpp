#include "krosc/nodes/nav2/behavior_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <thread>
#include <utility>

#include "krosc/math.hpp"
#include "krosc/qos.hpp"

namespace krosc::nodes::nav2 {

BehaviorServer::BehaviorServer(Context& context, std::string name,
                               std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      cmd_vel_topic_(declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_nav")),
      odom_topic_(declare_parameter<std::string>("odom_topic", "odom")),
      max_ang_(declare_parameter<double>("max_ang", 1.0, [](double value) { return value > 0.0; })),
      max_lin_(declare_parameter<double>("max_lin", 0.1, [](double value) { return value > 0.0; })) {
  odom_subscription_ = subscribe<msgs::Odometry>(
      odom_topic_.value(), [this](const msgs::Odometry& odom) { on_odom(odom); }, QOS_SENSOR_DATA);
  spin_action_ = std::make_unique<
      ActionServer<msgs::nav2::SpinGoal, msgs::nav2::SpinFeedback, msgs::nav2::SpinResult>>(
      context, msgs::nav2::SPIN_ACTION,
      [this](SpinHandle& handle) { return execute_spin(handle); });
  backup_action_ = std::make_unique<
      ActionServer<msgs::nav2::BackUpGoal, msgs::nav2::BackUpFeedback, msgs::nav2::BackUpResult>>(
      context, msgs::nav2::BACKUP_ACTION,
      [this](BackUpHandle& handle) { return execute_backup(handle); });
  wait_action_ = std::make_unique<
      ActionServer<msgs::nav2::WaitGoal, msgs::nav2::WaitFeedback, msgs::nav2::WaitResult>>(
      context, msgs::nav2::WAIT_ACTION,
      [this](WaitHandle& handle) { return execute_wait(handle); });
}

BehaviorServer::~BehaviorServer() {
  if (spin_action_) {
    spin_action_->shutdown();
  }
  if (backup_action_) {
    backup_action_->shutdown();
  }
  if (wait_action_) {
    wait_action_->shutdown();
  }
}

void BehaviorServer::on_odom(const msgs::Odometry& odom) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_x_ = odom.pose.pose.position.x;
  odom_y_ = odom.pose.pose.position.y;
  odom_yaw_ = tfmath::yaw_of(odom.pose.pose.orientation);
  has_odom_ = true;
}

void BehaviorServer::publish_velocity(double linear, double angular) {
  msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  publish(cmd_vel_topic_.value(), std::move(twist), QOS_COMMAND);
}

void BehaviorServer::stop() { publish_velocity(0.0, 0.0); }

msgs::nav2::SpinResult BehaviorServer::execute_spin(SpinHandle& handle) {
  msgs::nav2::SpinResult result;
  const double target = std::abs(handle.goal().target_yaw);
  const double began = now();
  const double sign = handle.goal().target_yaw >= 0.0 ? 1.0 : -1.0;
  double traveled = 0.0;
  std::optional<double> previous;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (has_odom_) {
      previous = odom_yaw_;
    }
  }
  for (;;) {
    if (handle.is_cancel_requested()) {
      stop();
      result.error_msg = "취소";
      return result;
    }
    if (now() - began >= handle.goal().time_allowance) {
      stop();
      result.error_code = msgs::nav2::SpinResult::TIMEOUT;
      result.error_msg = std::to_string(handle.goal().time_allowance) + "초 안에 회전하지 못했다";
      return result;
    }
    if (traveled >= target) {
      stop();
      return result;
    }
    publish_velocity(0.0, sign * max_ang_.value());
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (previous && has_odom_) {
        traveled += std::abs(std::atan2(std::sin(odom_yaw_ - *previous), std::cos(odom_yaw_ - *previous)));
        previous = odom_yaw_;
      }
    }
    msgs::nav2::SpinFeedback feedback;
    feedback.angular_distance_traveled = traveled;
    feedback.total_elapsed_time = now() - began;
    handle.publish_feedback(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

msgs::nav2::BackUpResult BehaviorServer::execute_backup(BackUpHandle& handle) {
  msgs::nav2::BackUpResult result;
  const double target = std::abs(handle.goal().backup_dist);
  const double began = now();
  double traveled = 0.0;
  std::optional<std::pair<double, double>> previous;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (has_odom_) {
      previous = std::make_pair(odom_x_, odom_y_);
    }
  }
  for (;;) {
    if (handle.is_cancel_requested()) {
      stop();
      result.error_msg = "취소";
      return result;
    }
    if (now() - began >= handle.goal().time_allowance) {
      stop();
      result.error_code = msgs::nav2::BackUpResult::TIMEOUT;
      result.error_msg = std::to_string(handle.goal().time_allowance) + "초 안에 후진하지 못했다";
      return result;
    }
    if (traveled >= target) {
      stop();
      return result;
    }
    publish_velocity(-std::abs(handle.goal().backup_speed), 0.0);
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (previous && has_odom_) {
        traveled += std::hypot(odom_x_ - previous->first, odom_y_ - previous->second);
        previous = std::make_pair(odom_x_, odom_y_);
      }
    }
    msgs::nav2::BackUpFeedback feedback;
    feedback.distance_traveled = traveled;
    feedback.total_elapsed_time = now() - began;
    handle.publish_feedback(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

msgs::nav2::WaitResult BehaviorServer::execute_wait(WaitHandle& handle) {
  msgs::nav2::WaitResult result;
  const double began = now();
  const double duration = handle.goal().wait_duration;
  while (now() - began < duration) {
    if (handle.is_cancel_requested()) {
      stop();
      result.error_msg = "취소";
      return result;
    }
    msgs::nav2::WaitFeedback feedback;
    feedback.time_left = std::max(0.0, duration - (now() - began));
    handle.publish_feedback(feedback);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  stop();
  return result;
}

}  // namespace krosc::nodes::nav2
