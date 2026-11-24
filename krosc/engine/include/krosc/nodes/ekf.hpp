#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"
#include "krosc/tf_ros.hpp"
#include "krosc/timer.hpp"
#include "krosc/topic.hpp"

namespace krosc::nodes {

inline constexpr const char* FILTERED_TOPIC_NAME = "odometry/filtered";

// robot_localization의 ekf_node 흉내 (2D, two_d_mode):
// 상태 [x, y, yaw, v, w]를 일정 속도 모델로 예측하고 바퀴 오도메트리/IMU로 고친다.
class Ekf final : public Node {
 public:
  explicit Ekf(Context& context, std::string name = "ekf_filter_node", std::map<std::string, ParamValue> parameters = {});

  [[nodiscard]] std::array<double, 5> state() const { return state_; }
  [[nodiscard]] std::uint64_t rejected() const { return rejected_.load(std::memory_order_relaxed); }
  [[nodiscard]] std::uint64_t late() const { return late_.load(std::memory_order_relaxed); }

  struct MeasurementItem {
    int state_index = 0;  // X=0, Y=1, YAW=2, V=3, W=4
    double value = 0.0;
    double variance = 0.0;
  };
  using Measurement = std::vector<MeasurementItem>;

 private:
  struct Pending {
    double stamp = 0.0;
    Measurement pose;
    Measurement twist;
    double pose_threshold = 0.0;
    double twist_threshold = 0.0;
  };

  void on_odom(const msgs::Odometry& message);
  void on_imu(const msgs::Imu& message);
  void on_timer();
  void enqueue(double stamp, Measurement pose, Measurement twist, double pose_threshold, double twist_threshold);
  [[nodiscard]] Measurement pick(const std::vector<bool>& config, const Measurement& values) const;
  void predict(double dt);
  bool correct(const Measurement& measurement, double threshold);
  void publish_filtered(double stamp);

  Parameter<double> frequency_;
  Parameter<std::string> odom_frame_;
  Parameter<std::string> base_link_frame_;
  Parameter<bool> publish_tf_;
  Parameter<std::string> odom0_;
  Parameter<std::vector<bool>> odom0_config_;
  Parameter<double> odom0_pose_rejection_threshold_;
  Parameter<double> odom0_twist_rejection_threshold_;
  Parameter<std::string> imu0_;
  Parameter<std::vector<bool>> imu0_config_;
  Parameter<bool> imu0_relative_;
  Parameter<double> imu0_pose_rejection_threshold_;
  Parameter<double> imu0_twist_rejection_threshold_;
  Parameter<std::vector<double>> process_noise_covariance_;
  Parameter<std::vector<double>> initial_estimate_covariance_;

  std::array<double, 5> state_{};
  std::array<double, 25> covariance_{};
  std::optional<double> time_;
  std::optional<double> imu_yaw_offset_;
  std::vector<Pending> queue_;
  std::mutex queue_mutex_;
  std::atomic<std::uint64_t> rejected_{0};
  std::atomic<std::uint64_t> late_{0};

  TransformBroadcaster broadcaster_;
  Subscription<msgs::Odometry> odom_subscription_;
  Subscription<msgs::Imu> imu_subscription_;
  Timer timer_;
};

}  // namespace krosc::nodes
