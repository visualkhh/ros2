#pragma once

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/tf.hpp"
#include "krosc/tf_ros.hpp"
#include "krosc/timer.hpp"
#include "krosc/topic.hpp"

#include "ml/onnx_policy.hpp"

namespace krosc_ws {

// Isaac rl_maze 정책(policy.onnx)을 20Hz로 실행해 cmd_vel_rl을 만든다.
// RL OFF/실패 시 cmd_vel_rl을 멈추고 기존 Navigator(pure pursuit, cmd_vel_nav)가 fallback이 된다.
class RlNavController final : public krosc::Node {
 public:
  explicit RlNavController(krosc::Context& context, std::string name = "rl_nav_controller",
                           std::map<std::string, krosc::ParamValue> parameters = {});

 private:
  struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  void on_scan(const krosc::msgs::LaserScan& message);
  void on_imu(const krosc::msgs::Imu& message);
  void on_odom(const krosc::msgs::Odometry& message);
  void on_plan(const krosc::msgs::Path& message);
  void on_enable(const krosc::msgs::Bool& message);
  void on_fallback(const krosc::msgs::String& message);
  void on_reload(const krosc::msgs::Empty& message);
  void tick();

  bool load_policy(std::string& error);
  [[nodiscard]] std::optional<Pose2D> robot_pose() const;
  [[nodiscard]] double laser_yaw(const std::string& scan_frame) const;
  [[nodiscard]] std::vector<float> waypoints(const Pose2D& pose, const krosc::msgs::Path& plan) const;
  [[nodiscard]] std::vector<float> scan_rays(const krosc::msgs::LaserScan& scan, double yaw,
                                             double inflation) const;
  void publish_status_locked(int reason, bool active, double v, double w, double a0, double a1,
                             double inference_ms);
  void publish_zero_locked();

  // parameters
  krosc::Parameter<std::string> model_path_;
  krosc::Parameter<bool> enable_default_;
  krosc::Parameter<std::string> fallback_default_;
  krosc::Parameter<std::int64_t> lookahead_;
  krosc::Parameter<double> wp_step_;
  krosc::Parameter<double> wp_scale_;
  krosc::Parameter<std::int64_t> lidar_rays_;
  krosc::Parameter<double> lidar_max_;
  krosc::Parameter<double> lidar_inflation_;
  krosc::Parameter<double> inflation_scale_;
  krosc::Parameter<double> max_lin_;
  krosc::Parameter<double> max_ang_;
  krosc::Parameter<double> v_min_ratio_;
  krosc::Parameter<double> frequency_;
  krosc::Parameter<double> scan_timeout_;
  krosc::Parameter<double> odom_timeout_;
  krosc::Parameter<std::string> map_frame_;
  krosc::Parameter<std::string> robot_frame_;

  mutable std::mutex mutex_;
  ml::OnnxPolicy policy_;
  std::string policy_error_;
  bool enable_ = false;
  std::string fallback_ = "pursuit";

  std::shared_ptr<const krosc::msgs::LaserScan> scan_;
  std::shared_ptr<const krosc::msgs::Imu> imu_;
  std::shared_ptr<const krosc::msgs::Odometry> odom_;
  std::shared_ptr<const krosc::msgs::Path> plan_;
  double scan_time_ = -1e18;
  double imu_time_ = -1e18;
  double odom_time_ = -1e18;
  double plan_time_ = -1e18;
  std::array<float, 2> prev_action_{{0.0F, 0.0F}};
  bool active_last_ = false;

  krosc::Buffer tf_buffer_;
  std::unique_ptr<krosc::TransformListener> tf_listener_;

  krosc::Subscription<krosc::msgs::LaserScan> scan_subscription_;
  krosc::Subscription<krosc::msgs::Imu> imu_subscription_;
  krosc::Subscription<krosc::msgs::Odometry> odom_subscription_;
  krosc::Subscription<krosc::msgs::Path> plan_subscription_;
  krosc::Subscription<krosc::msgs::Bool> enable_subscription_;
  krosc::Subscription<krosc::msgs::String> fallback_subscription_;
  krosc::Subscription<krosc::msgs::Empty> reload_subscription_;
  krosc::Timer timer_;
};

}  // namespace krosc_ws
