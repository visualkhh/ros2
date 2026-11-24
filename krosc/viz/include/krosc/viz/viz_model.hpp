#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf.hpp"
#include "krosc/tf_ros.hpp"

namespace krosc::viz {

// viz가 구독하는 최신 값과 토픽 통계. 구독 콜백(시뮬레이션/실행 스레드)과 화면(Qt 스레드)이
// 락 안에서 shared_ptr만 맞바꾸므로 그리는 쪽은 복사 없이 읽는다.
class VizModel {
 public:
  struct TopicRow {
    std::string topic;
    std::string type;
    std::uint64_t count = 0;
    double hz = 0.0;
    bool recent = false;
  };

  explicit VizModel(Context& context);
  ~VizModel();

  VizModel(const VizModel&) = delete;
  VizModel& operator=(const VizModel&) = delete;

  [[nodiscard]] Node& node() { return node_; }
  [[nodiscard]] Buffer& tf_buffer() { return buffer_; }

  void set_fixed_frame(std::string frame);
  [[nodiscard]] std::string fixed_frame() const { return fixed_frame_; }

  [[nodiscard]] std::shared_ptr<const msgs::LaserScan> scan() const;
  [[nodiscard]] std::shared_ptr<const msgs::Imu> imu() const;
  [[nodiscard]] std::shared_ptr<const msgs::Odometry> odom() const;
  [[nodiscard]] std::shared_ptr<const msgs::Odometry> filtered() const;
  [[nodiscard]] std::shared_ptr<const msgs::OccupancyGrid> map() const;
  [[nodiscard]] std::shared_ptr<const msgs::OccupancyGrid> global_costmap() const;
  [[nodiscard]] std::shared_ptr<const msgs::OccupancyGrid> local_costmap() const;
  [[nodiscard]] std::shared_ptr<const msgs::Path> plan() const;
  [[nodiscard]] std::shared_ptr<const msgs::Path> local_plan() const;
  [[nodiscard]] std::shared_ptr<const msgs::DiagnosticArray> diagnostics() const;
  [[nodiscard]] std::shared_ptr<const msgs::String> robot_description() const;
  [[nodiscard]] std::shared_ptr<const msgs::String> map_list() const;
  [[nodiscard]] std::shared_ptr<const msgs::String> map_status() const;
  [[nodiscard]] std::shared_ptr<const msgs::String> goal_status() const;
  [[nodiscard]] std::shared_ptr<const msgs::String> collision_status() const;
  [[nodiscard]] std::shared_ptr<const msgs::String> arm_status() const;
  [[nodiscard]] std::shared_ptr<const msgs::String> rl_nav_status() const;
  [[nodiscard]] std::shared_ptr<const msgs::String> moveit_status() const;
  [[nodiscard]] std::shared_ptr<const msgs::String> moveit_state() const;
  [[nodiscard]] std::shared_ptr<const msgs::PoseStamped> moveit_target_pose() const;
  [[nodiscard]] std::shared_ptr<const msgs::Path> moveit_arm() const;
  [[nodiscard]] std::shared_ptr<const msgs::Path> moveit_path() const;
  [[nodiscard]] std::shared_ptr<const msgs::JointState> joint_states() const;
  [[nodiscard]] std::shared_ptr<const msgs::String> slam_status() const;
  [[nodiscard]] std::vector<msgs::Log> logs() const;

  [[nodiscard]] std::vector<TopicRow> topic_rows() const;

  // frame의 점을 fixed_frame에서 본 자세/변환. TF가 아직 없으면 nullopt.
  [[nodiscard]] std::optional<msgs::PoseStamped> pose_in_fixed(
      const std::string& frame, std::optional<double> stamp = std::nullopt) const;
  [[nodiscard]] std::optional<msgs::TransformStamped> transform_in_fixed(const std::string& frame) const;

  void publish_cmd_vel(double linear, double angular);
  void publish_goal(double x, double y, double theta);

 private:
  struct Stat {
    std::string type;
    std::uint64_t count = 0;
    double first = 0.0;
    double last = 0.0;
  };

  template <class T>
  void subscribe_latest(const std::string& topic, std::shared_ptr<const T>& target, QoS qos) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_[topic] = Stat{std::string(msgs::message_type_name<T>()), 0, 0.0, 0.0};
    }
    node_.subscribe<T>(
        topic,
        [this, topic, &target](const T& message) {
          auto value = std::make_shared<const T>(message);
          const double now = monotonic_now();
          std::lock_guard<std::mutex> lock(mutex_);
          target = std::move(value);
          auto& stat = stats_[topic];
          ++stat.count;
          if (stat.count == 1) {
            stat.first = now;
          }
          stat.last = now;
        },
        qos);
  }

  void subscribe_stats_only(const std::string& topic, const std::string& type, QoS qos);
  void store_log(const msgs::Log& log);
  // 여러 노드가 각자 diagnostics를 내므로 이름 기준으로 합친다 (diagnostic_aggregator 흉내).
  void store_diagnostics(const msgs::DiagnosticArray& message);
  [[nodiscard]] static double monotonic_now();

  Node node_;
  Buffer buffer_;
  TransformListener listener_;

  mutable std::mutex mutex_;
  std::map<std::string, Stat> stats_;
  std::shared_ptr<const msgs::LaserScan> scan_;
  std::shared_ptr<const msgs::Imu> imu_;
  std::shared_ptr<const msgs::Odometry> odom_;
  std::shared_ptr<const msgs::Odometry> filtered_;
  std::shared_ptr<const msgs::OccupancyGrid> map_;
  std::shared_ptr<const msgs::OccupancyGrid> global_costmap_;
  std::shared_ptr<const msgs::OccupancyGrid> local_costmap_;
  std::shared_ptr<const msgs::Path> plan_;
  std::shared_ptr<const msgs::Path> local_plan_;
  std::shared_ptr<const msgs::DiagnosticArray> diagnostics_;
  std::shared_ptr<const msgs::String> robot_description_;
  std::shared_ptr<const msgs::String> map_list_;
  std::shared_ptr<const msgs::String> map_status_;
  std::shared_ptr<const msgs::String> goal_status_;
  std::shared_ptr<const msgs::String> collision_status_;
  std::shared_ptr<const msgs::String> arm_status_;
  std::shared_ptr<const msgs::String> rl_nav_status_;
  std::shared_ptr<const msgs::String> moveit_status_;
  std::shared_ptr<const msgs::String> moveit_state_;
  std::shared_ptr<const msgs::PoseStamped> moveit_target_pose_;
  std::shared_ptr<const msgs::Path> moveit_arm_;
  std::shared_ptr<const msgs::Path> moveit_path_;
  std::shared_ptr<const msgs::JointState> joint_states_;
  std::shared_ptr<const msgs::String> slam_status_;
  std::deque<msgs::Log> logs_;
  std::string fixed_frame_ = "map";
};

}  // namespace krosc::viz
