#pragma once

#include <random>

#include "krosc/nodes/nav2/controller.hpp"

namespace krosc::nodes::nav2 {

// nav2_mppi_controller::MPPIController 흉내: 지난 제어 시퀀스에 잡음을 섞은 후보를 굴려 critic 비용을 매기고,
// exp(-비용/temperature) 가중 평균 시퀀스의 첫 명령을 낸다.
class MPPIController final : public NavControllerPlugin {
 public:
  void configure(Node& node, const std::string& name) override;
  Velocity compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                     const std::vector<Pose2>& path, const Costmap& costmap,
                                     double dt) override;
  [[nodiscard]] std::string type() const override { return "nav2_mppi_controller::MPPIController"; }

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
