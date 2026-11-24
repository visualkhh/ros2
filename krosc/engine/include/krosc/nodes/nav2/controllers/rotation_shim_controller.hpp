#pragma once

#include <memory>
#include <string>
#include <vector>

#include "krosc/nodes/nav2/controllers/controller.hpp"

namespace krosc::nodes::nav2 {

// nav2_rotation_shim_controller 흉내: 새 경로의 초기 방향과 로봇 yaw 차이가 크면
// 제자리에서 돌고(회전 중에는 선속도 0), threshold 안이면 primary controller(RPP 기본)에 넘긴다.
// 들어갈 때 angular_dist_threshold, 빠져나올 때 angular_disengage_threshold를 써서 떨림을 막는다.
class RotationShimController final : public NavControllerPlugin {
 public:
  void configure(Node& node, const std::string& name) override;
  Velocity compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                     const std::vector<Pose2>& path, const Costmap& costmap,
                                     double dt) override;
  [[nodiscard]] std::string type() const override {
    return "nav2_rotation_shim_controller::RotationShimController";
  }

 private:
  struct Sample {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    bool valid = false;
  };

  [[nodiscard]] Sample sample_path(const std::vector<Pose2>& path) const;
  Velocity rotate_to_heading(double angular_distance, const Pose2& pose,
                             const Velocity& velocity, const Costmap& costmap, double dt);
  void check_collision(const Costmap& costmap, const Pose2& pose, double angular_velocity,
                       double angular_distance, double dt) const;

  Parameter<double> angular_dist_threshold_;
  Parameter<double> angular_disengage_threshold_;
  Parameter<double> forward_sampling_distance_;
  Parameter<double> rotate_to_heading_angular_vel_;
  Parameter<double> max_angular_accel_;
  Parameter<double> max_cost_threshold_;
  Parameter<double> simulate_ahead_time_;
  Parameter<bool> use_path_orientations_;
  Parameter<bool> closed_loop_;
  Parameter<std::string> primary_plugin_;

  std::unique_ptr<NavControllerPlugin> primary_;
  bool in_rotation_ = false;
  double last_angular_vel_ = 0.0;
};

}  // namespace krosc::nodes::nav2
