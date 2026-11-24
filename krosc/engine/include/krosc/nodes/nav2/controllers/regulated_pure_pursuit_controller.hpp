#pragma once

#include "krosc/nodes/nav2/controllers/controller.hpp"

namespace krosc::nodes::nav2 {

// nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController 흉내:
// 경로 위 lookahead 거리의 점(carrot)으로 가는 원호의 곡률로 회전 속도를 정하고, 급커브/목표 근처에서 앞 속도를 줄인다.
class RegulatedPurePursuitController final : public NavControllerPlugin {
 public:
  void configure(Node& node, const std::string& name) override;
  Velocity compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                     const std::vector<Pose2>& path, const Costmap& costmap,
                                     double dt) override;
  [[nodiscard]] std::string type() const override { return "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"; }

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

}  // namespace krosc::nodes::nav2
