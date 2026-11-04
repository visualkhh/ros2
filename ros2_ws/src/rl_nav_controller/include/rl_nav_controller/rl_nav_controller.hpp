// RlNavController — nav2 controller 플러그인. 학습한 주행 정책(isaac/training/rl_maze, policy.mlp)으로 경로를 따라간다.
// ~/toggle (std_srvs/SetBool): false(기본) = 안에 품은 기본 controller(MPPI 등) 그대로 · true = 정책.
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rl_nav_controller/mlp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace rl_nav_controller
{

class RlNavController : public nav2_core::Controller
{
public:
  void configure(
    const nav2::LifecycleNode::WeakPtr & parent, std::string name,
    nav2::TransformBuffer::SharedPtr tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void newPathReceived(const nav_msgs::msg::Path & raw_global_path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & transformed_global_plan,
    const geometry_msgs::msg::PoseStamped & global_goal) override;
  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

  // 관측 (학습 env와 같은 순서 · 크기) — 테스트용으로 공개
  std::vector<float> waypoints(const geometry_msgs::msg::PoseStamped & pose, const nav_msgs::msg::Path & plan) const;
  std::vector<float> scan_rays(const sensor_msgs::msg::LaserScan & scan, double laser_yaw) const;

private:
  void load_model(const std::string & path);

  nav2::LifecycleNode::WeakPtr node_;
  std::string name_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("RlNavController")};

  pluginlib::ClassLoader<nav2_core::Controller> loader_{"nav2_core", "nav2_core::Controller"};
  nav2_core::Controller::Ptr inner_;   // 바이패스 때 쓰는 기본 controller
  Mlp mlp_;

  nav2::ServiceServer<std_srvs::srv::SetBool>::SharedPtr toggle_srv_;
  nav2::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  nav2::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::mutex mtx_;
  bool enable_ = false;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  double gyro_z_ = 0.0;
  bool gyro_seen_ = false;
  double laser_yaw_ = 0.0;
  bool laser_yaw_known_ = false;
  float prev_a_[2] = {0.0f, 0.0f};

  // 파라미터 (기본값 = isaac/training/rl_maze/env_cfg.py · robot_cfg.py — 학습과 같아야 한다)
  int lookahead_ = 5;
  double wp_step_ = 0.30, wp_scale_ = 2.0;
  int lidar_rays_ = 36;
  double lidar_max_ = 3.5;
  double max_lin_ = 0.26, max_ang_ = 1.82, v_min_ratio_ = -0.2;
  double speed_ratio_ = 1.0;
};

}  // namespace rl_nav_controller
