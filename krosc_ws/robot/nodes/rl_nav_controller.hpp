#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/nodes/nav2/controllers/controller.hpp"
#include "krosc/tf.hpp"
#include "krosc/tf_ros.hpp"
#include "krosc/timer.hpp"
#include "krosc/topic.hpp"

#include "ml/onnx_policy.hpp"

namespace krosc_ws {

// nav2 controller 플러그인: ONNX 정책이 가능하면 cmd_vel_nav를 RL로 내고,
// 꺼져 있거나 정책/스캔이 없으면 inner fallback(mppi/dwb/rpp)에 위임한다.
// ROS 2/Python 예제의 rl_nav_controller와 같은 위치(ControllerServer 플러그인)다.
class RlNavController final : public krosc::nodes::nav2::NavControllerPlugin {
 public:
  using Pose2 = krosc::nodes::nav2::Pose2;
  using Velocity = krosc::nodes::nav2::Velocity;
  using Costmap = krosc::nodes::nav2::Costmap;

  RlNavController();
  ~RlNavController() override;

  void configure(krosc::Node& node, const std::string& name) override;
  Velocity compute_velocity_commands(const Pose2& pose, const Velocity& velocity,
                                     const std::vector<Pose2>& path, const Costmap& costmap,
                                     double dt) override;
  [[nodiscard]] std::string type() const override {
    return "rl_nav_controller::RlNavController";
  }

 private:
  void on_scan(const krosc::msgs::LaserScan& message);
  void on_imu(const krosc::msgs::Imu& message);
  void on_enable(const krosc::msgs::Bool& message);
  void on_fallback(const krosc::msgs::String& message);
  void on_reload(const krosc::msgs::Empty& message);
  void on_model(const krosc::msgs::String& message);

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
  [[nodiscard]] std::vector<float> scan_rays(const krosc::msgs::LaserScan& scan, double laser_yaw,
                                             double inflation) const;
  double laser_yaw();
  [[nodiscard]] double scan_min() const;

  // parameters ("<name>." 아래)
  krosc::Parameter<bool> enable_;
  krosc::Parameter<std::string> model_path_;
  krosc::Parameter<std::string> models_directory_;
  krosc::Parameter<std::string> model_;
  krosc::Parameter<std::string> fallback_controller_;
  krosc::Parameter<double> max_lin_;
  krosc::Parameter<double> max_ang_;
  krosc::Parameter<double> v_min_ratio_;
  krosc::Parameter<std::int64_t> lookahead_;
  krosc::Parameter<double> wp_step_;
  krosc::Parameter<double> wp_scale_;
  krosc::Parameter<std::int64_t> lidar_rays_;
  krosc::Parameter<double> lidar_max_;
  krosc::Parameter<double> lidar_inflation_;
  krosc::Parameter<double> inflation_scale_;
  krosc::Parameter<double> scan_timeout_;
  krosc::Parameter<std::string> base_frame_;
  krosc::Parameter<std::string> scan_topic_;
  krosc::Parameter<std::string> imu_topic_;
  krosc::Parameter<double> diagnostics_period_;

  mutable std::mutex mutex_;
  std::unique_ptr<krosc::nodes::nav2::NavControllerPlugin> fallback_;
  ml::OnnxPolicy policy_;
  std::string policy_error_;
  std::string current_model_;
  std::vector<std::string> model_names_;
  std::shared_ptr<const krosc::msgs::LaserScan> scan_;
  double scan_time_ = -1e18;
  std::optional<double> gyro_;
  std::array<float, 2> prev_action_{{0.0F, 0.0F}};
  bool active_last_ = false;
  double last_status_ = -1e18;
  double last_diagnostics_ = -1e18;
  double last_model_list_ = -1e18;
  krosc::Timer status_timer_;

  krosc::Buffer buffer_;
  std::unique_ptr<krosc::TransformListener> listener_;
  std::optional<double> laser_yaw_;

  krosc::Subscription<krosc::msgs::LaserScan> scan_subscription_;
  krosc::Subscription<krosc::msgs::Imu> imu_subscription_;
  krosc::Subscription<krosc::msgs::Bool> enable_subscription_;
  krosc::Subscription<krosc::msgs::String> fallback_subscription_;
  krosc::Subscription<krosc::msgs::Empty> reload_subscription_;
  krosc::Subscription<krosc::msgs::String> model_subscription_;
};

}  // namespace krosc_ws
