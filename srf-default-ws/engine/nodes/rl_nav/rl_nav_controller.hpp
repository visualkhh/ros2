#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "srf/msgs.hpp"
#include "srf/node.hpp"
#include "srf/nodes/nav2/controllers/controller.hpp"
#include "srf/tf/tf.hpp"
#include "srf/tf/tf_ros.hpp"
#include "srf/time/timer.hpp"
#include "srf/topic.hpp"

#include "ml/onnx_policy.hpp"

namespace srf_default_ws {

// nav2 controller 플러그인: ONNX 정책이 가능하면 cmd_vel_nav를 RL로 내고,
// 꺼져 있거나 정책/스캔이 없으면 inner fallback(mppi/dwb/rpp)에 위임한다.
// ROS 2/Python 예제의 rl_nav_controller와 같은 위치(ControllerServer 플러그인)다.
class RlNavController final : public srf::nodes::nav2::NavControllerPlugin {
 public:
  using Pose2 = srf::nodes::nav2::Pose2;
  using Velocity = srf::nodes::nav2::Velocity;
  using Costmap = srf::nodes::nav2::Costmap;

  RlNavController();
  ~RlNavController() override;

  void configure(srf::Node& node, const std::string& name) override;
  Velocity compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                     const std::vector<Pose2>& path, const Costmap& costmap,
                                     double dt) override;
  [[nodiscard]] std::string type() const override {
    return "rl_nav_controller::RlNavController";
  }

 private:
  void on_scan(const srf::msgs::LaserScan& message);
  void on_imu(const srf::msgs::Imu& message);
  void on_enable_value(bool enabled);
  void on_fallback_value(const std::string& fallback);
  void on_model_value(const std::string& model);
  srf::msgs::TriggerResponse on_reload();

  bool set_fallback_locked(const std::string& kind);
  bool load_policy_locked(const std::string& model, std::string& error);
  void refresh_models_locked();
  void publish_model_list_locked();
  void publish_status_locked(const Velocity& command, int reason, bool active, double inference_ms);
  void publish_diagnostics_locked(const Velocity& command, int reason, bool active,
                                  double inference_ms);
  void cancel_subscriptions();

  [[nodiscard]] std::vector<float> waypoints(const Pose2& pose,
                                             const std::vector<Pose2>& path) const;
  [[nodiscard]] std::vector<float> scan_rays(const srf::msgs::LaserScan& scan, double laser_yaw,
                                             double inflation) const;
  double laser_yaw();
  [[nodiscard]] double scan_min() const;

  // parameters ("<name>." 아래)
  srf::Parameter<bool> enable_;
  srf::Parameter<std::string> model_path_;
  srf::Parameter<std::string> models_directory_;
  srf::Parameter<std::string> model_;
  srf::Parameter<std::string> fallback_controller_;
  srf::Parameter<double> max_lin_;
  srf::Parameter<double> max_ang_;
  srf::Parameter<double> v_min_ratio_;
  srf::Parameter<std::int64_t> lookahead_;
  srf::Parameter<double> wp_step_;
  srf::Parameter<double> wp_scale_;
  srf::Parameter<std::int64_t> lidar_rays_;
  srf::Parameter<double> lidar_max_;
  srf::Parameter<double> lidar_inflation_;
  srf::Parameter<double> inflation_scale_;
  srf::Parameter<double> scan_timeout_;
  srf::Parameter<std::string> base_frame_;
  srf::Parameter<std::string> scan_topic_;
  srf::Parameter<std::string> imu_topic_;
  srf::Parameter<double> diagnostics_period_;

  mutable std::mutex mutex_;
  std::unique_ptr<srf::nodes::nav2::NavControllerPlugin> fallback_;
  ml::OnnxPolicy policy_;
  std::string policy_error_;
  std::string current_model_;
  // 정책 입력 크기에서 역산한 관측 구성 (모델마다 lidar ray 수가 다를 수 있다).
  std::int64_t active_rays_ = 0;
  // 폴백 컨트롤러를 다시 만들 때 파라미터 이름이 겹치지 않게 하는 인스턴스 번호.
  std::int64_t fallback_instance_ = 0;
  bool inflation_feature_ = false;
  std::vector<std::string> model_names_;
  std::shared_ptr<const srf::msgs::LaserScan> scan_;
  double scan_time_ = -1e18;
  std::optional<double> gyro_;
  std::array<float, 2> prev_action_{{0.0F, 0.0F}};
  bool active_last_ = false;
  double last_status_ = -1e18;
  double last_diagnostics_ = -1e18;
  double last_model_list_ = -1e18;
  srf::Timer status_timer_;

  srf::Buffer buffer_;
  std::unique_ptr<srf::TransformListener> listener_;
  std::optional<double> laser_yaw_;

  srf::Subscription<srf::msgs::LaserScan> scan_subscription_;
  srf::Subscription<srf::msgs::Imu> imu_subscription_;
  srf::ServiceServer reload_service_;
};

}  // namespace srf_default_ws
