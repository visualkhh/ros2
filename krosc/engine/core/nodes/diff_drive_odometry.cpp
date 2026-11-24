#include "krosc/nodes/diff_drive_odometry.hpp"

#include <cmath>
#include <cstdint>
#include <utility>

namespace krosc::nodes {

namespace {

constexpr double kStoppedVariance = 1e-6;

std::array<double, 36> covariance_matrix(const std::vector<double>& diagonal) {
  std::array<double, 36> matrix{};
  for (std::size_t i = 0; i < 6 && i < diagonal.size(); ++i) {
    matrix[i * 6 + i] = diagonal[i];
  }
  return matrix;
}

bool valid_diagonal(const std::vector<double>& value) {
  if (value.size() != 6) {
    return false;
  }
  for (double item : value) {
    if (item < 0.0) {
      return false;
    }
  }
  return true;
}

}  // namespace

DiffDriveOdometry::DiffDriveOdometry(Context& context, std::string name,
                                     std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      wheel_radius_(declare_parameter<double>("wheel_radius", 0.033, [](double value) { return value > 0.0; })),
      wheel_separation_(
          declare_parameter<double>("wheel_separation", 0.288, [](double value) { return value > 0.0; })),
      left_joint_(declare_parameter<std::string>("left_joint", "wheel_left_joint")),
      right_joint_(declare_parameter<std::string>("right_joint", "wheel_right_joint")),
      odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
      base_frame_(declare_parameter<std::string>("base_frame", "base_footprint")),
      publish_tf_(declare_parameter<bool>("publish_tf", true)),
      jump_threshold_(declare_parameter<double>("jump_threshold", 1.0, [](double value) { return value > 0.0; })),
      pose_covariance_diagonal_(declare_parameter<std::vector<double>>(
          "pose_covariance_diagonal", {0.001, 0.001, 1e6, 1e6, 1e6, 0.01}, valid_diagonal)),
      twist_covariance_diagonal_(declare_parameter<std::vector<double>>(
          "twist_covariance_diagonal", {0.001, 0.001, 1e6, 1e6, 1e6, 0.01}, valid_diagonal)),
      broadcaster_(*this) {
  subscription_ = subscribe<msgs::JointState>(JOINT_STATES_TOPIC_NAME,
                                             [this](const msgs::JointState& message) { on_joint_states(message); });
}

void DiffDriveOdometry::on_joint_states(const msgs::JointState& message) {
  double left = 0.0;
  double right = 0.0;
  bool has_left = false;
  bool has_right = false;
  for (std::size_t i = 0; i < message.name.size() && i < message.position.size(); ++i) {
    if (message.name[i] == left_joint_.value()) {
      left = message.position[i];
      has_left = true;
    } else if (message.name[i] == right_joint_.value()) {
      right = message.position[i];
      has_right = true;
    }
  }
  if (!has_left || !has_right) {
    return;  // 이 로봇의 바퀴가 아닌 joint_states
  }
  const double stamp = message.header.stamp;
  double linear = 0.0;
  double angular = 0.0;
  if (!has_previous_) {
    has_previous_ = true;
    last_change_ = stamp;
    has_last_change_ = true;
  } else {
    const double dl = left - previous_left_;
    const double dr = right - previous_right_;
    const double dt = stamp - previous_stamp_;
    if (std::abs(dl) > jump_threshold_.value() || std::abs(dr) > jump_threshold_.value()) {
      ++jumps_;
      logger().warn("바퀴 각이 한 번에 (%+.2f, %+.2f) rad 뛰었다: 그 변화는 버린다 (보드 재부팅?)", dl, dr);
      last_change_ = stamp;
      last_linear_ = 0.0;
      last_angular_ = 0.0;
      has_last_change_ = true;
    } else if (std::abs(dl) > 1e-9 || std::abs(dr) > 1e-9) {
      double interval = stamp - (has_last_change_ ? last_change_ : previous_stamp_);
      if (interval <= 0.0) {
        interval = dt;
      }
      const double radius = wheel_radius_.value();
      const double separation = wheel_separation_.value();
      const double distance = radius * (dl + dr) / 2.0;
      const double turn = radius * (dr - dl) / separation;
      x_ += distance * std::cos(yaw_ + turn / 2.0);
      y_ += distance * std::sin(yaw_ + turn / 2.0);
      yaw_ += turn;
      linear = interval > 0.0 ? distance / interval : 0.0;
      angular = interval > 0.0 ? turn / interval : 0.0;
      last_change_ = stamp;
      last_linear_ = linear;
      last_angular_ = angular;
      has_last_change_ = true;
    } else if (has_last_change_ && stamp - last_change_ <= 0.3) {
      linear = last_linear_;
      angular = last_angular_;
    }
  }
  previous_stamp_ = stamp;
  previous_left_ = left;
  previous_right_ = right;
  publish_odometry(stamp, linear, angular);
}

void DiffDriveOdometry::publish_odometry(double stamp, double linear, double angular) {
  msgs::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = odom_frame_.value();
  odometry.child_frame_id = base_frame_.value();
  odometry.pose.pose.position = {x_, y_, 0.0};
  odometry.pose.pose.orientation = tfmath::from_yaw(yaw_);
  odometry.pose.covariance = covariance_matrix(pose_covariance_diagonal_.value());
  std::vector<double> twist_diagonal = twist_covariance_diagonal_.value();
  if (linear == 0.0 && angular == 0.0) {
    if (twist_diagonal.size() == 6) {
      twist_diagonal[0] = kStoppedVariance;
      twist_diagonal[1] = kStoppedVariance;
      twist_diagonal[5] = kStoppedVariance;
    }
  }
  odometry.twist.twist.linear.x = linear;
  odometry.twist.twist.angular.z = angular;
  odometry.twist.covariance = covariance_matrix(twist_diagonal);
  publish(ODOM_TOPIC_NAME, std::move(odometry));

  if (publish_tf_.value()) {
    msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_.value();
    transform.child_frame_id = base_frame_.value();
    transform.transform.translation = {x_, y_, 0.0};
    transform.transform.rotation = tfmath::from_yaw(yaw_);
    broadcaster_.send_transform(transform);
  }
}

}  // namespace krosc::nodes
