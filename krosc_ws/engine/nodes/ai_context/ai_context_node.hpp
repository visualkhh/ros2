#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/tf/tf.hpp"
#include "krosc/tf/tf_ros.hpp"

namespace krosc_ws {

// URDF에서 계산한 정적 치수. LLM/VLA/VLM이 도달 가능성과 통과 가능성을
// 판단할 때 사용한다. TF가 알려주는 실시간 카메라/TCP 위치와 별개다.
struct RobotGeometry {
  std::array<double, 3> footprint{0.266, 0.266, 0.141};  // x, y, base 높이
  std::array<double, 3> arm_base{-0.092, 0.0, 0.091};    // base_link 기준
  double arm_reach = 0.449;                              // arm_base에서 손끝까지
  bool from_urdf = false;
};

// AI 서버 헬스 체크 대상. URL이 비어 있거나 enabled=false면 검사하지 않는다.
struct AiServerConfig {
  bool llm_enabled = false;
  std::string llm_url;
  std::string llm_model;
  bool vlm_enabled = false;
  std::string vlm_url;
  bool vla_enabled = false;
  std::string vla_url;
};

struct AiServerHealth {
  bool enabled = false;
  bool ok = false;
  double checked_at = 0.0;
  std::string detail = "not checked";
};

// LLM/VLA/VLM이 행동 전에 참고할 AI 상태 스냅샷.
// 소유권: 하드웨어 상태/진단은 RobotStatus, 자세/속도는 센서/컨트롤러,
// TF는 robot_state_publisher가 소유한다. 이 노드는 그 canonical 토픽들을
// 읽어 하나의 JSON으로 조합만 하며 OpenCR/LDS/카메라를 직접 제어하지 않는다.
// topic: robot_context (String JSON, 5Hz)
class AiContextNode final : public krosc::Node {
 public:
  AiContextNode(krosc::Context& context, std::string scan_topic,
                std::string color_topic, std::string depth_topic,
                RobotGeometry geometry = {}, AiServerConfig servers = {});
  ~AiContextNode() override;

  AiContextNode(const AiContextNode&) = delete;
  AiContextNode& operator=(const AiContextNode&) = delete;

 private:
  void update();
  void health_loop();
  void probe_servers();

  std::string scan_topic_;
  std::string color_topic_;
  std::string depth_topic_;
  RobotGeometry geometry_;
  AiServerConfig servers_;

  std::atomic<bool> health_stop_{false};
  std::thread health_thread_;
  mutable std::mutex health_mutex_;
  AiServerHealth llm_health_;
  AiServerHealth vlm_health_;
  AiServerHealth vla_health_;

  mutable std::mutex state_mutex_;
  bool odom_seen_ = false;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;
  double odom_linear_ = 0.0;
  double odom_angular_ = 0.0;
  double odom_stamp_ = 0.0;

  bool scan_seen_ = false;
  double scan_stamp_ = 0.0;
  double scan_front_ = 0.0;
  double scan_left_ = 0.0;
  double scan_right_ = 0.0;
  double scan_rear_ = 0.0;

  bool color_seen_ = false;
  double color_stamp_ = 0.0;
  unsigned color_width_ = 0;
  unsigned color_height_ = 0;
  bool depth_seen_ = false;
  double depth_stamp_ = 0.0;
  unsigned depth_width_ = 0;
  unsigned depth_height_ = 0;
  double depth_min_ = 0.0;
  double depth_median_ = 0.0;

  bool joints_seen_ = false;
  double joints_stamp_ = 0.0;
  std::vector<double> arm_joints_;
  double gripper_ = 0.0;
  bool arm_status_seen_ = false;
  double arm_status_stamp_ = 0.0;
  bool arm_present_ = false;
  std::vector<double> arm_temperature_;
  std::vector<double> arm_current_;
  std::vector<int> arm_error_;
  bool wheel_seen_ = false;
  double wheel_left_pos_ = 0.0;
  double wheel_right_pos_ = 0.0;
  double wheel_left_vel_ = 0.0;
  double wheel_right_vel_ = 0.0;
  double wheel_left_temperature_ = 0.0;
  double wheel_right_temperature_ = 0.0;
  double wheel_left_current_ = 0.0;
  double wheel_right_current_ = 0.0;

  std::string opencr_message_;
  double battery_voltage_ = 0.0;
  double battery_percent_ = 0.0;

  krosc::Buffer tf_buffer_;
  std::unique_ptr<krosc::TransformListener> tf_listener_;

  krosc::Subscription<krosc::msgs::Odometry> odom_subscription_;
  krosc::Subscription<krosc::msgs::LaserScan> scan_subscription_;
  krosc::Subscription<krosc::msgs::JointState> joint_subscription_;
  krosc::Subscription<krosc::msgs::String> arm_status_subscription_;
  krosc::Subscription<krosc::msgs::DiagnosticArray> diagnostics_subscription_;
  krosc::Subscription<krosc::msgs::Image> color_subscription_;
  krosc::Subscription<krosc::msgs::Image> depth_subscription_;
  krosc::Timer timer_;
};

}  // namespace krosc_ws
