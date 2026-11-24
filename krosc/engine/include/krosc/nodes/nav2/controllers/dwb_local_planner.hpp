#pragma once

#include "krosc/nodes/nav2/controllers/controller.hpp"

namespace krosc::nodes::nav2 {

// dwb_core::DWBLocalPlanner 흉내 (DWA 계열): 가속 한계 안의 (v, w) 표본을 sim_time 동안 굴려 critic 점수의 합이 가장 낮은 것을 고른다.
class DWBLocalPlanner final : public NavControllerPlugin {
 public:
  void configure(Node& node, const std::string& name) override;
  Velocity compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                     const std::vector<Pose2>& path, const Costmap& costmap,
                                     double dt) override;
  [[nodiscard]] std::string type() const override { return "dwb_core::DWBLocalPlanner"; }

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

}  // namespace krosc::nodes::nav2
