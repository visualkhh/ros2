#pragma once

#include <array>
#include <map>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/tf/tf_ros.hpp"
#include "krosc/topic.hpp"

namespace krosc::nodes {

inline constexpr const char* ODOM_TOPIC_NAME = "odom";

// 차동 구동 오도메트리: joint_states의 바퀴 회전각을 적분해 odom 좌표계의 로봇 위치를 추정한다.
// odom 토픽과 (publish_tf=True면) odom -> base_frame TF를 낸다.
class DiffDriveOdometry final : public Node {
 public:
  explicit DiffDriveOdometry(Context& context, std::string name = "diff_drive_odometry",
                             std::map<std::string, ParamValue> parameters = {});

  [[nodiscard]] double x() const { return x_; }
  [[nodiscard]] double y() const { return y_; }
  [[nodiscard]] double yaw() const { return yaw_; }
  [[nodiscard]] std::uint64_t jumps() const { return jumps_; }

 private:
  void on_joint_states(const msgs::JointState& message);
  void publish_odometry(double stamp, double linear, double angular);

  Parameter<double> wheel_radius_;
  Parameter<double> wheel_separation_;
  Parameter<std::string> left_joint_;
  Parameter<std::string> right_joint_;
  Parameter<std::string> odom_frame_;
  Parameter<std::string> base_frame_;
  Parameter<bool> publish_tf_;
  Parameter<double> jump_threshold_;
  Parameter<std::vector<double>> pose_covariance_diagonal_;
  Parameter<std::vector<double>> twist_covariance_diagonal_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  std::uint64_t jumps_ = 0;
  bool has_previous_ = false;
  double previous_stamp_ = 0.0;
  double previous_left_ = 0.0;
  double previous_right_ = 0.0;
  bool has_last_change_ = false;
  double last_change_ = 0.0;
  double last_linear_ = 0.0;
  double last_angular_ = 0.0;
  TransformBroadcaster broadcaster_;
  Subscription<msgs::JointState> subscription_;
};

}  // namespace krosc::nodes
