// RlNavController — 관측 · 행동은 isaac/training/rl_maze/env.py와 같은 약속 (바꾸면 같이):
//   관측 51 = [다음 웨이포인트 K(5)개 로봇 기준 (x, y) ÷ wp_scale · 라이다 R(36)방향 ÷ lidar_max · v ÷ max_lin · w ÷ max_ang · 자이로 w ÷ max_ang · 이전 행동 2]
//   행동 [-1, 1]² → v = a0 × max_lin (후진은 v_min_ratio까지) · w = a1 × max_ang
#include "rl_nav_controller/rl_nav_controller.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_ros_common/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rl_nav_controller
{

template<typename T>
static T param(const nav2::LifecycleNode::SharedPtr & node, const std::string & name, const T & def)
{
  nav2::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(def));
  T v = def;
  node->get_parameter(name, v);
  return v;
}

void RlNavController::configure(
  const nav2::LifecycleNode::WeakPtr & parent, std::string name,
  nav2::TransformBuffer::SharedPtr tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  auto node = parent.lock();
  logger_ = node->get_logger();

  const auto inner = param<std::string>(node, name + ".inner_plugin", "nav2_mppi_controller::MPPIController");
  const auto model = param<std::string>(node, name + ".model_path", "");
  enable_ = param<bool>(node, name + ".enable", false);
  const auto scan_topic = param<std::string>(node, name + ".scan_topic", "scan");
  const auto imu_topic = param<std::string>(node, name + ".imu_topic", "imu");
  lookahead_ = param<int>(node, name + ".lookahead", lookahead_);
  wp_step_ = param<double>(node, name + ".wp_step", wp_step_);
  wp_scale_ = param<double>(node, name + ".wp_scale", wp_scale_);
  lidar_rays_ = param<int>(node, name + ".lidar_rays", lidar_rays_);
  lidar_max_ = param<double>(node, name + ".lidar_max", lidar_max_);
  max_lin_ = param<double>(node, name + ".max_lin", max_lin_);
  max_ang_ = param<double>(node, name + ".max_ang", max_ang_);
  v_min_ratio_ = param<double>(node, name + ".v_min_ratio", v_min_ratio_);

  // 바이패스용 기본 controller — 파라미터는 <name>.inner.* (예: FollowPath.inner.time_steps)
  inner_ = loader_.createSharedInstance(inner);
  inner_->configure(parent, name + ".inner", tf, costmap_ros);
  load_model(model);
  if (enable_ && !mlp_.loaded()) {
    RCLCPP_WARN(logger_, "[%s] enable인데 정책 없음 — 바이패스로 (%s)", name.c_str(), inner.c_str());
    enable_ = false;
  }

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_);
      scan_ = m;
    }, nav2::qos::SensorDataQoS());
  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, [this](sensor_msgs::msg::Imu::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_);
      gyro_z_ = m->angular_velocity.z;
      gyro_seen_ = true;
    }, nav2::qos::SensorDataQoS());
  toggle_srv_ = node->create_service<std_srvs::srv::SetBool>(
    "~/" + name + "/toggle",
    [this](const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      std::lock_guard<std::mutex> lk(mtx_);
      if (req->data && !mlp_.loaded()) {
        res->success = false;
        res->message = "정책 없음 (model_path) — 바이패스 그대로";
        return;
      }
      if (enable_ != req->data && !req->data) {inner_->reset();}   // 정책 → 기본 controller로 돌아갈 때 깨끗하게
      enable_ = req->data;
      prev_a_[0] = prev_a_[1] = 0.0f;
      res->success = true;
      res->message = std::string("enable=") + (enable_ ? "true (정책)" : "false (바이패스)") +
        " · model=" + (mlp_.loaded() ? "loaded" : "none");
      RCLCPP_INFO(logger_, "[%s] toggle: %s", name_.c_str(), res->message.c_str());
    });
  RCLCPP_INFO(
    logger_, "[%s] RlNavController: inner=%s · model=%s · enable=%s · toggle: ~/%s/toggle",
    name.c_str(), inner.c_str(), mlp_.loaded() ? model.c_str() : "(없음)", enable_ ? "true" : "false", name.c_str());
}

void RlNavController::load_model(const std::string & path)
{
  if (path.empty()) {return;}
  try {
    Mlp m;
    m.load(path);
    const uint32_t want = uint32_t(2 * lookahead_ + lidar_rays_ + 5);
    if (m.inputs() != want || m.outputs() != 2) {
      RCLCPP_ERROR(logger_, "정책 입력 %u · 출력 %u — 기대 %u · 2 (lookahead · lidar_rays 확인): %s",
        m.inputs(), m.outputs(), want, path.c_str());
      return;
    }
    mlp_ = std::move(m);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
  }
}

void RlNavController::cleanup() {inner_->cleanup(); scan_sub_.reset(); imu_sub_.reset(); toggle_srv_.reset();}
void RlNavController::activate() {inner_->activate();}
void RlNavController::deactivate() {inner_->deactivate();}
void RlNavController::newPathReceived(const nav_msgs::msg::Path & raw) {inner_->newPathReceived(raw);}
bool RlNavController::cancel() {return inner_->cancel();}
void RlNavController::reset() {inner_->reset(); prev_a_[0] = prev_a_[1] = 0.0f;}

void RlNavController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  inner_->setSpeedLimit(speed_limit, percentage);
  speed_ratio_ = percentage ? speed_limit / 100.0 : speed_limit / max_lin_;
  speed_ratio_ = (speed_limit <= 0.0) ? 1.0 : std::clamp(speed_ratio_, 0.0, 1.0);   // 0 = 제한 없음 (nav2 약속)
}

std::vector<float> RlNavController::waypoints(
  const geometry_msgs::msg::PoseStamped & pose, const nav_msgs::msg::Path & plan) const
{
  // 경로(pose와 같은 frame)에서 로봇에 가장 가까운 점부터 wp_step 간격으로 앞의 K개 → 로봇 기준 (x 앞, y 왼쪽) ÷ wp_scale
  std::vector<float> out(2 * lookahead_, 0.0f);
  const auto & ps = plan.poses;
  if (ps.empty()) {return out;}
  const double px = pose.pose.position.x, py = pose.pose.position.y, yaw = tf2::getYaw(pose.pose.orientation);
  size_t near = 0;
  double best = 1e18;
  for (size_t i = 0; i < ps.size(); ++i) {
    const double d = std::hypot(ps[i].pose.position.x - px, ps[i].pose.position.y - py);
    if (d < best) {best = d; near = i;}
  }
  const double c = std::cos(yaw), s = std::sin(yaw);
  double acc = 0.0, want = wp_step_;
  size_t i = near;
  int k = 0;
  auto emit = [&](double x, double y) {
      const double dx = x - px, dy = y - py;
      out[2 * k] = float((c * dx + s * dy) / wp_scale_);
      out[2 * k + 1] = float((-s * dx + c * dy) / wp_scale_);
      ++k;
    };
  while (k < lookahead_ && i + 1 < ps.size()) {
    const auto & a = ps[i].pose.position, & b = ps[i + 1].pose.position;
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    while (k < lookahead_ && acc + seg >= want) {
      const double u = seg > 1e-9 ? (want - acc) / seg : 0.0;
      emit(a.x + u * (b.x - a.x), a.y + u * (b.y - a.y));
      want += wp_step_;
    }
    acc += seg;
    ++i;
  }
  while (k < lookahead_) {emit(ps.back().pose.position.x, ps.back().pose.position.y);}   // 끝 = 마지막 점 (학습 env와 같음)
  return out;
}

std::vector<float> RlNavController::scan_rays(const sensor_msgs::msg::LaserScan & scan, double laser_yaw) const
{
  // 로봇 기준 0°부터 반시계로 R방향 — 각 방향에서 가장 가까운 빔(±1)의 거리, 없으면 lidar_max → ÷ lidar_max
  std::vector<float> out(lidar_rays_, 1.0f);
  const int n = int(scan.ranges.size());
  if (n == 0 || scan.angle_increment == 0.0f) {return out;}
  for (int r = 0; r < lidar_rays_; ++r) {
    double a = 2.0 * M_PI * r / lidar_rays_ - laser_yaw - scan.angle_min;
    a = std::fmod(std::fmod(a, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
    const int idx = int(std::lround(a / scan.angle_increment));
    double best = lidar_max_;
    for (int j = idx - 1; j <= idx + 1; ++j) {
      const float v = scan.ranges[((j % n) + n) % n];
      if (std::isfinite(v) && v >= scan.range_min && v <= scan.range_max) {best = std::min(best, double(v));}
    }
    out[r] = float(std::clamp(best, 0.0, lidar_max_) / lidar_max_);
  }
  return out;
}

geometry_msgs::msg::TwistStamped RlNavController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & global_goal)
{
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  bool on;
  double gyro;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    on = enable_ && mlp_.loaded();
    scan = scan_;
    gyro = gyro_seen_ ? gyro_z_ : velocity.angular.z;
  }
  if (!on || !scan) {
    if (on) {RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 2000, "[%s] /scan 없음 — 바이패스", name_.c_str());}
    return inner_->computeVelocityCommands(pose, velocity, goal_checker, transformed_global_plan, global_goal);
  }
  if (!laser_yaw_known_) {   // 라이다가 로봇 기준으로 얼마나 돌아 있는지 (한 번)
    try {
      const auto t = tf_->lookupTransform(costmap_ros_->getBaseFrameID(), scan->header.frame_id, tf2::TimePointZero);
      laser_yaw_ = tf2::getYaw(t.transform.rotation);
      laser_yaw_known_ = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 2000, "라이다 tf 없음 (%s) — 0°로", e.what());
    }
  }
  std::vector<float> obs = waypoints(pose, transformed_global_plan);
  const auto rays = scan_rays(*scan, laser_yaw_);
  obs.insert(obs.end(), rays.begin(), rays.end());
  obs.push_back(float(velocity.linear.x / max_lin_));
  obs.push_back(float(velocity.angular.z / max_ang_));
  obs.push_back(float(gyro / max_ang_));
  obs.push_back(prev_a_[0]);
  obs.push_back(prev_a_[1]);
  const auto a = mlp_.forward(obs);
  const float a0 = std::clamp(a[0], -1.0f, 1.0f), a1 = std::clamp(a[1], -1.0f, 1.0f);
  prev_a_[0] = a0;
  prev_a_[1] = a1;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = pose.header.stamp;
  cmd.twist.linear.x = std::max(double(a0) * max_lin_, v_min_ratio_ * max_lin_) * speed_ratio_;
  cmd.twist.angular.z = double(a1) * max_ang_;
  return cmd;
}

}  // namespace rl_nav_controller

PLUGINLIB_EXPORT_CLASS(rl_nav_controller::RlNavController, nav2_core::Controller)
