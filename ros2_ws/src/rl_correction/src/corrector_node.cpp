// Movement correction RL node (/rl_corrector) — Isaac Lab에서 학습한 ONNX 정책 (ONNX Runtime C++, rl_policy).
//
// 두 모드 (ONNX 입력 차원으로 자동 판별):
//   track (obs 6)  [v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change] → [-1, 1] → clamp × action_scale → [dV, dW]
//                  /cmd_vel_in → 보정 → /cmd_vel_out
//   goal (obs 10)  [gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_a0, prev_a1] — isaac/training/rl_correction_waypoint와 같은 약속
//                  자세 = 시작 자세 기준 추정 (거리 = 엔코더 /odom twist · 방향 = 자이로 /imu 적분, 20 Hz)
//                  /rl_goal (Point)을 받으면 nominal controller + ONNX 보정 → /cmd_vel_out
//                  goal_mode=absolute: stop 전까지 첫 goal의 시작 자세 고정 · relative: goal마다 현재 자세가 시작
// 바이패스 (enable false · 모델 없음): track = 입력 그대로 통과 · goal = nominal controller만 (보정 0, A/B의 A).
// goal 모델이어도 웨이포인트가 없을 때 /cmd_vel_in은 그대로 통과 (calibration 네모 · 키보드).
//
// ONNX 적용 켬/끔 (기본 켬): ros2 service call /rl_corrector/toggle std_srvs/srv/SetBool "{data: false}"
//   = ros2 param set /rl_corrector enable false|true · model_path <onnx> = 즉시 다시 로드 (없는 경로 · 빈 값 = 바이패스)
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rl_correction/correct.hpp"
#include "rl_policy/onnx_policy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;
using geometry_msgs::msg::Twist;

namespace rl_correction
{

constexpr double T_CHANGE_MAX = 2.0;   // [s] sim과 같은 clip
constexpr double CHANGE_EPS = 1e-3;    // 명령이 바뀐 것으로 보는 차이

class RlCorrector : public rclcpp::Node
{
public:
  RlCorrector()
  : Node("rl_corrector")
  {
    declare_parameter("enable", true);   // ONNX 적용 (false = 바이패스) — ~/toggle 서비스로도
    declare_parameter("model_path", "");
    declare_parameter("mode", "auto");   // auto | track | goal
    declare_parameter("goal_mode", "absolute");   // absolute (웨이포인트, 학습과 같음) | relative
    declare_parameter("max_lin", 0.26);
    declare_parameter("max_ang", 1.82);
    declare_parameter("goal_tol", 0.05);   // goal 모드 (학습 cfg와 같은 기본값)
    declare_parameter("k_dist", 1.0);
    declare_parameter("k_heading", 2.0);
    declare_parameter("turn_thresh", 0.5);
    declare_parameter("action_scale_lin", 0.10);
    declare_parameter("action_scale_ang", 0.60);
    declare_parameter("log_period", 1.0);   // [s] 0이면 매 스텝
    log_ms_ = std::max(0.0, get_parameter("log_period").as_double()) * 1000.0;

    load_model(get_parameter("model_path").as_string());
    post_cb_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
        for (const auto & p : ps) {
          if (p.get_name() == "model_path") {load_model(p.as_string());}
        }
      });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        v_enc_ = m->twist.twist.linear.x;
        w_enc_ = m->twist.twist.angular.z;
        odom_seen_ = true;
      });
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>("imu", 10, [this](sensor_msgs::msg::Imu::SharedPtr m) {
        w_gyro_ = m->angular_velocity.z;
      });
    sub_cmd_ = create_subscription<Twist>("cmd_vel_in", 10, [this](Twist::SharedPtr m) {cb_cmd(*m);});
    sub_goal_ = create_subscription<geometry_msgs::msg::Point>(
      "rl_goal", 10, [this](geometry_msgs::msg::Point::SharedPtr m) {cb_goal(*m);});
    sub_stop_ = create_subscription<std_msgs::msg::Empty>("rl_goal_stop", 10, [this](std_msgs::msg::Empty::SharedPtr) {
        goal_active_ = false;
        start_pose_ = false;
        pub_->publish(Twist());
        RCLCPP_INFO(get_logger(), "goal stop");
      });
    pub_ = create_publisher<Twist>("cmd_vel_out", 10);
    pub_dbg_ = create_publisher<std_msgs::msg::Float32MultiArray>("rl_debug", 10);
    timer_ = create_wall_timer(50ms, [this]() {tick_goal();});
    srv_ = create_service<std_srvs::srv::SetBool>(
      "~/toggle", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        set_parameter(rclcpp::Parameter("enable", bool(req->data)));
        res->success = true;
        res->message = std::string("enable=") + (req->data ? "True" : "False") +
        " · model=" + (policy_.loaded() ? "loaded" : "none (bypass)") + " · mode=" + mode_;
        RCLCPP_INFO(get_logger(), "toggle: %s", res->message.c_str());
      });
  }

private:
  double p(const char * name) {return get_parameter(name).as_double();}
  bool enabled() {return get_parameter("enable").as_bool() && policy_.loaded();}

  void info_throttled(const std::string & s)
  {
    if (log_ms_ <= 0.0) {RCLCPP_INFO(get_logger(), "%s", s.c_str()); return;}
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), int64_t(log_ms_), "%s", s.c_str());
  }

  void load_model(const std::string & path)
  {
    obs_dim_ = 0;
    policy_.unload();   // 실패 · 빈 경로 = 바이패스
    if (path.empty()) {
      mode_ = "track";   // 모델 내림 — 입력 그대로 통과
      RCLCPP_INFO(get_logger(), "No model_path: passthrough mode");
      return;
    }
    try {
      policy_.load(path);
      obs_dim_ = int(policy_.inputs());
      const auto req = get_parameter("mode").as_string();
      mode_ = (req == "auto") ? (obs_dim_ == 10 ? "goal" : "track") : req;
      if (mode_ == "goal" && obs_dim_ != 10) {RCLCPP_WARN(get_logger(), "goal mode인데 obs_dim=%d (10이어야 함)", obs_dim_);}
      if (mode_ == "track" && obs_dim_ != 6) {RCLCPP_WARN(get_logger(), "track mode인데 obs_dim=%d (6이어야 함)", obs_dim_);}
      RCLCPP_INFO(
        get_logger(), "ONNX loaded: %s (obs_dim=%d, mode=%s, action_scale=(%.3f, %.3f), goal_mode=%s, log_period=%.2fs)",
        path.c_str(), obs_dim_, mode_.c_str(), p("action_scale_lin"), p("action_scale_ang"),
        get_parameter("goal_mode").as_string().c_str(), log_ms_ / 1000.0);
    } catch (const std::exception & e) {   // 노드는 살아서 바이패스
      RCLCPP_WARN(get_logger(), "ONNX load failed (%s): passthrough", e.what());
    }
  }

  std::pair<double, double> infer_delta(const std::vector<float> & obs, bool keep_prev)
  {
    const auto out = policy_.run(obs);
    if (keep_prev) {prev_a_ = {std::clamp(double(out[0]), -1.0, 1.0), std::clamp(double(out[1]), -1.0, 1.0)};}
    return scale_action(out[0], out[1], p("action_scale_lin"), p("action_scale_ang"));
  }

  void publish(double lin, double ang, const std::vector<float> & dbg)
  {
    Twist out;
    out.linear.x = lin;
    out.angular.z = ang;
    pub_->publish(out);
    std_msgs::msg::Float32MultiArray d;
    d.data = dbg;
    pub_dbg_->publish(d);
  }

  // ------------------------------------------------------------------ track
  void cb_cmd(const Twist & msg)
  {
    if (mode_ != "track") {   // goal 모델: 웨이포인트 중이면 goal 제어가 출력 · 아니면 속도 명령 그대로 통과
      if (!goal_active_) {pub_->publish(msg);}
      return;
    }
    const std::pair<double, double> cmd{msg.linear.x, msg.angular.z};
    const auto now = std::chrono::steady_clock::now();
    if (std::fabs(cmd.first - last_cmd_.first) > CHANGE_EPS || std::fabs(cmd.second - last_cmd_.second) > CHANGE_EPS) {
      changed_at_ = now;
    }
    last_cmd_ = cmd;
    std::pair<double, double> delta{0.0, 0.0};
    if (enabled()) {
      const double t_change = std::min(std::chrono::duration<double>(now - changed_at_).count(), T_CHANGE_MAX);
      const std::vector<float> obs{float(cmd.first), float(cmd.second), float(v_enc_), float(w_enc_), float(w_gyro_),
        float(t_change)};
      if (int(obs.size()) == obs_dim_) {
        delta = infer_delta(obs, false);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "obs dim %zu != model %d: passthrough", obs.size(), obs_dim_);
      }
    }
    const auto [lin, ang] = correct(cmd, delta, p("max_lin"), p("max_ang"));
    publish(lin, ang, {0.0f, float(cmd.first), float(cmd.second), float(v_enc_), float(w_enc_), float(w_gyro_), 0.0f,
        0.0f, float(cmd.first), float(cmd.second), float(delta.first), float(delta.second), float(lin), float(ang),
        float(lin - cmd.first), float(ang - cmd.second)});
    char b[320];
    std::snprintf(
      b, sizeof(b), "[track] in=(%+.3f,%+.3f) odom=(%+.3f,%+.3f) gyro=%+.3f model=(%+.3f,%+.3f) out=(%+.3f,%+.3f) "
      "applied=(%+.3f,%+.3f)", cmd.first, cmd.second, v_enc_, w_enc_, w_gyro_, delta.first, delta.second, lin, ang,
      lin - cmd.first, ang - cmd.second);
    info_throttled(b);
  }

  // ------------------------------------------------------------------ goal
  void cb_goal(const geometry_msgs::msg::Point & msg)
  {
    if (mode_ != "goal") {RCLCPP_WARN(get_logger(), "/rl_goal 무시: mode=%s", mode_.c_str()); return;}
    if (!odom_seen_) {RCLCPP_WARN(get_logger(), "/rl_goal 무시: /odom 아직 없음"); return;}
    goal_ = {msg.x, msg.y};
    if (get_parameter("goal_mode").as_string() != "absolute" || !start_pose_) {   // absolute: stop 전까지 첫 시작 자세 고정
      start_pose_ = true;
      est_ = {0.0, 0.0, 0.0};
      est_t_ = std::chrono::steady_clock::now();
    }
    goal_active_ = true;
    prev_a_ = {0.0, 0.0};
    RCLCPP_INFO(get_logger(), "goal 시작: g=(%+.2f, %+.2f) m frame=%s", goal_->first, goal_->second,
      get_parameter("goal_mode").as_string().c_str());
  }

  void rel_pose()   // 학습 env와 같은 식: 방향 += 자이로 · dt, 위치 += 엔코더 v · (중간 방향)
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = est_t_ ? std::min(0.2, std::chrono::duration<double>(now - *est_t_).count()) : 0.0;
    est_t_ = now;
    const double yaw_mid = est_[2] + 0.5 * w_gyro_ * dt;
    est_[0] += v_enc_ * std::cos(yaw_mid) * dt;
    est_[1] += v_enc_ * std::sin(yaw_mid) * dt;
    est_[2] = wrap_to_pi(est_[2] + w_gyro_ * dt);
  }

  void tick_goal()
  {
    if (mode_ != "goal" || !goal_active_) {return;}
    if (!odom_seen_ || !start_pose_ || !goal_) {pub_->publish(Twist()); return;}
    rel_pose();
    const double px = est_[0], py = est_[1], yaw = est_[2], gx = goal_->first, gy = goal_->second;
    const double dist = std::hypot(gx - px, gy - py), herr = wrap_to_pi(std::atan2(gy - py, gx - px) - yaw);
    double v = std::min(p("max_lin"), p("k_dist") * dist);   // nominal controller (= 학습 env)
    double w = std::clamp(p("k_heading") * herr, -p("max_ang"), p("max_ang"));
    if (std::fabs(herr) > p("turn_thresh") || dist < p("goal_tol")) {v = 0.0;}
    if (dist < p("goal_tol")) {w = 0.0;}

    std::pair<double, double> delta{0.0, 0.0};
    if (enabled()) {   // ONNX 보정
      const std::vector<float> obs{float(gx), float(gy), float(px), float(py), float(yaw), float(v_enc_), float(w_enc_),
        float(w_gyro_), float(prev_a_.first), float(prev_a_.second)};
      if (int(obs.size()) != obs_dim_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "obs dim %zu != model %d: stop", obs.size(), obs_dim_);
        pub_->publish(Twist());
        return;
      }
      delta = infer_delta(obs, true);
    } else {   // 바이패스 — nominal controller만 (보정 0 = 학습 env의 행동 0)
      prev_a_ = {0.0, 0.0};
    }
    const auto [lin, ang] = correct({v, w}, delta, p("max_lin"), p("max_ang"));
    publish(lin, ang, {1.0f, float(gx), float(gy), float(px), float(py), float(yaw), float(dist), float(herr), float(v),
        float(w), float(delta.first), float(delta.second), float(lin), float(ang), float(lin - v), float(ang - w)});
    char b[360];
    std::snprintf(
      b, sizeof(b), "[goal] g=(%+.2f,%+.2f) p=(%+.2f,%+.2f,%+.2f) dist=%.3f herr=%+.2f nominal=(%+.3f,%+.3f) "
      "model=(%+.3f,%+.3f) out=(%+.3f,%+.3f) applied=(%+.3f,%+.3f)", gx, gy, px, py, yaw, dist, herr, v, w, delta.first,
      delta.second, lin, ang, lin - v, ang - w);
    info_throttled(b);
    if (dist < p("goal_tol")) {
      goal_active_ = false;
      pub_->publish(Twist());
      RCLCPP_INFO(get_logger(), "goal 도착: err=%.3f m, p=(%+.2f, %+.2f)", dist, px, py);
    }
  }

  rl_policy::OnnxPolicy policy_;
  int obs_dim_ = 0;
  std::string mode_ = "track";
  double log_ms_ = 1000.0;
  double v_enc_ = 0.0, w_enc_ = 0.0, w_gyro_ = 0.0;
  bool odom_seen_ = false, start_pose_ = false, goal_active_ = false;
  std::array<double, 3> est_{0.0, 0.0, 0.0};   // 시작 자세 기준 (x, y, yaw)
  std::optional<std::chrono::steady_clock::time_point> est_t_;
  std::optional<std::pair<double, double>> goal_;
  std::pair<double, double> prev_a_{0.0, 0.0}, last_cmd_{0.0, 0.0};
  std::chrono::steady_clock::time_point changed_at_ = std::chrono::steady_clock::now();

  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_cb_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_dbg_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
};

}  // namespace rl_correction

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rl_correction::RlCorrector>());
  rclcpp::shutdown();
  return 0;
}
