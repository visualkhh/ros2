#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "krosc/nodes/nav2/controller.hpp"

namespace krosc::nodes::nav2 {

// nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController 흉내:
// 경로 위 lookahead 거리의 점(carrot)으로 가는 원호의 곡률로 회전 속도를 정하고, 급커브/목표 근처에서 앞 속도를 줄인다.
class RegulatedPurePursuitController final : public Controller {
 public:
  void configure(Node& node, const std::string& name) override;
  Velocity compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                     const std::vector<Pose2>& path, const Costmap& costmap,
                                     double dt) override;

  [[nodiscard]] const std::vector<Pose2>& carrot() const { return carrot_; }

 private:
  double rotate(double angle, double current, double dt) const;

  Parameter<double> desired_linear_vel_;
  Parameter<double> lookahead_dist_;
  Parameter<double> min_lookahead_dist_;
  Parameter<double> max_lookahead_dist_;
  Parameter<double> lookahead_time_;
  Parameter<bool> use_velocity_scaled_lookahead_dist_;
  Parameter<bool> use_rotate_to_heading_;
  Parameter<double> rotate_to_heading_angular_vel_;
  Parameter<double> rotate_to_heading_min_angle_;
  Parameter<double> max_angular_accel_;
  Parameter<bool> use_regulated_linear_velocity_scaling_;
  Parameter<double> regulated_linear_scaling_min_radius_;
  Parameter<double> regulated_linear_scaling_min_speed_;
  Parameter<double> approach_velocity_scaling_dist_;
  Parameter<double> min_approach_linear_velocity_;
  Parameter<bool> use_collision_detection_;
  Parameter<double> max_allowed_time_to_collision_up_to_carrot_;
  Parameter<double> goal_dist_tol_;
  std::vector<Pose2> carrot_;
};

// dwb_core::DWBLocalPlanner 흉내 (DWA 계열): 가속 한계 안의 (v, w) 표본을 sim_time 동안 굴려 critic 점수의 합이 가장 낮은 것을 고른다.
class DWBLocalPlanner final : public Controller {
 public:
  void configure(Node& node, const std::string& name) override;
  Velocity compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                     const std::vector<Pose2>& path, const Costmap& costmap,
                                     double dt) override;

 private:
  [[nodiscard]] std::vector<Pose2> pruned_path(const Pose2& pose, const std::vector<Pose2>& path) const;

  Parameter<double> min_vel_x_;
  Parameter<double> max_vel_x_;
  Parameter<double> max_vel_theta_;
  Parameter<double> acc_lim_x_;
  Parameter<double> acc_lim_theta_;
  Parameter<double> decel_lim_x_;
  Parameter<double> decel_lim_theta_;
  Parameter<std::int64_t> vx_samples_;
  Parameter<std::int64_t> vtheta_samples_;
  Parameter<double> sim_time_;
  Parameter<double> prune_distance_;
  Parameter<double> xy_goal_tolerance_;
  Parameter<double> rotate_to_goal_scale_;
  Parameter<double> base_obstacle_scale_;
  Parameter<double> goal_align_scale_;
  Parameter<double> path_align_scale_;
  Parameter<double> path_dist_scale_;
  Parameter<double> goal_dist_scale_;
  Parameter<double> path_align_forward_;
  Parameter<double> goal_align_forward_;
};

// nav2_mppi_controller::MPPIController 흉내: 지난 제어 시퀀스에 잡음을 섞은 후보를 굴려 critic 비용을 매기고,
// exp(-비용/temperature) 가중 평균 시퀀스의 첫 명령을 낸다.
class MPPIController final : public Controller {
 public:
  void configure(Node& node, const std::string& name) override;
  Velocity compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                     const std::vector<Pose2>& path, const Costmap& costmap,
                                     double dt) override;

 private:
  double score(const std::vector<Pose2>& trajectory, const std::vector<Velocity>& controls,
               const Costmap& costmap, const std::vector<Pose2>& local, const Pose2& goal,
               double to_goal, const Pose2& follow, const Pose2& ahead, const Pose2& pose,
               double model_dt) const;

  Parameter<std::int64_t> batch_size_;
  Parameter<std::int64_t> time_steps_;
  Parameter<double> model_dt_;
  Parameter<double> vx_std_;
  Parameter<double> wz_std_;
  Parameter<double> vx_max_;
  Parameter<double> vx_min_;
  Parameter<double> wz_max_;
  Parameter<double> temperature_;
  Parameter<std::int64_t> smoothing_window_;
  Parameter<std::int64_t> seed_;
  Parameter<double> cost_critic_weight_;
  Parameter<double> goal_critic_weight_;
  Parameter<double> goal_angle_critic_weight_;
  Parameter<double> path_align_critic_weight_;
  Parameter<double> path_follow_critic_weight_;
  Parameter<double> path_angle_critic_weight_;
  Parameter<double> prefer_forward_critic_weight_;
  Parameter<double> goal_threshold_;
  Parameter<double> goal_angle_threshold_;
  Parameter<double> path_follow_threshold_;
  Parameter<std::int64_t> path_follow_offset_;
  Parameter<std::int64_t> path_angle_offset_;
  Parameter<double> max_angle_to_furthest_;
  std::mt19937 rng_;
  std::vector<Velocity> control_;
};

}  // namespace krosc::nodes::nav2
