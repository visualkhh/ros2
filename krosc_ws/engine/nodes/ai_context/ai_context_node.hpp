#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/tf/tf.hpp"
#include "krosc/tf/tf_ros.hpp"

namespace krosc_ws {

// LLM/VLA/VLM이 행동 전에 참고할 AI 상태 스냅샷.
// 소유권: 하드웨어 상태/진단은 RobotStatus, 자세/속도는 센서/컨트롤러,
// TF는 robot_state_publisher가 소유한다. 이 노드는 그 canonical 토픽들을
// 읽어 하나의 JSON으로 조합만 하며 OpenCR/LDS/카메라를 직접 제어하지 않는다.
// topic: robot_context (String JSON, 5Hz)
class AiContextNode final : public krosc::Node {
 public:
  AiContextNode(krosc::Context& context, std::string scan_topic,
                std::string color_topic, std::string depth_topic);
  ~AiContextNode() override;

  AiContextNode(const AiContextNode&) = delete;
  AiContextNode& operator=(const AiContextNode&) = delete;

 private:
  void update();

  std::string scan_topic_;
  std::string color_topic_;
  std::string depth_topic_;

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
  bool wheel_seen_ = false;
  double wheel_left_pos_ = 0.0;
  double wheel_right_pos_ = 0.0;
  double wheel_left_vel_ = 0.0;
  double wheel_right_vel_ = 0.0;

  std::string opencr_message_;
  double battery_voltage_ = 0.0;
  double battery_percent_ = 0.0;

  krosc::Buffer tf_buffer_;
  std::unique_ptr<krosc::TransformListener> tf_listener_;

  krosc::Subscription<krosc::msgs::Odometry> odom_subscription_;
  krosc::Subscription<krosc::msgs::LaserScan> scan_subscription_;
  krosc::Subscription<krosc::msgs::JointState> joint_subscription_;
  krosc::Subscription<krosc::msgs::DiagnosticArray> diagnostics_subscription_;
  krosc::Subscription<krosc::msgs::Image> color_subscription_;
  krosc::Subscription<krosc::msgs::Image> depth_subscription_;
  krosc::Timer timer_;
};

}  // namespace krosc_ws
