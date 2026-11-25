#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "krosc/cartographer/probability_grid.hpp"
#include "krosc/cartographer/scan_matcher.hpp"
#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/tf.hpp"
#include "krosc/tf_ros.hpp"

namespace krosc::nodes {

// cartographer_ros 흉내 (2D): 라이다 스캔을 격자 지도에 맞춰(스캔 매칭) 로봇 위치를 바로잡고,
// map -> odom TF를 낸다. odom -> base는 EKF가 내고 이 노드는 그 위에 map -> odom 하나만 얹는다.
//
// mode: idle / mapping / localization. mapping은 새 지도를 만들며 map 토픽(latch)으로 내고,
// localization은 map_server가 낸 지도에 맞춰 위치만 고친다. /initialpose(2D Pose Estimate)를 받는다.
class Cartographer final : public Node {
 public:
  explicit Cartographer(Context& context, std::string name = "cartographer_node",
                        std::map<std::string, ParamValue> parameters = {});
  ~Cartographer() override = default;

  Cartographer(const Cartographer&) = delete;
  Cartographer& operator=(const Cartographer&) = delete;

  [[nodiscard]] std::string mode() const;
  [[nodiscard]] int scans() const;
  [[nodiscard]] int rejected() const;
  [[nodiscard]] bool localized() const;
  [[nodiscard]] std::size_t map_cells() const;
  [[nodiscard]] cartographer::Pose2 map_to_odom() const;
  [[nodiscard]] std::string slam_status() const;

 private:
  void on_scan(const msgs::LaserScan& scan);
  void on_map(const msgs::OccupancyGrid& map);
  void on_initial_pose(const msgs::PoseWithCovarianceStamped& message);
  void publish_map();
  void publish_tf();
  void set_mode(const std::string& mode);
  [[nodiscard]] bool moved(const cartographer::Pose2& pose) const;  // mutex를 잡은 채로 부른다

  // 파라미터
  Parameter<std::string> map_frame_;
  Parameter<std::string> odom_frame_;
  Parameter<std::string> tracking_frame_;
  Parameter<std::string> scan_topic_;
  Parameter<double> resolution_;
  Parameter<double> map_publish_period_sec_;
  Parameter<double> tf_publish_period_sec_;
  Parameter<double> linear_search_window_;
  Parameter<double> angular_search_window_;
  Parameter<double> min_score_;
  Parameter<double> motion_filter_max_distance_;
  Parameter<double> motion_filter_max_angle_;
  Parameter<double> tf_smoothing_;
  Parameter<std::int64_t> num_matching_points_;
  Parameter<std::int64_t> match_every_;
  Parameter<std::string> mode_;

  // 상태
  mutable std::mutex mutex_;
  std::string mode_str_;  // ""로 시작해 첫 set_mode에서 slam_status를 반드시 발행한다
  std::unique_ptr<cartographer::ProbabilityGrid> grid_;
  std::optional<msgs::OccupancyGrid> external_map_;
  cartographer::Pose2 map_to_odom_{0.0, 0.0, 0.0};
  cartographer::Pose2 last_inserted_{0.0, 0.0, 0.0};
  cartographer::Pose2 last_match_odom_{0.0, 0.0, 0.0};
  cartographer::Pose2 tf_published_{0.0, 0.0, 0.0};
  bool has_last_inserted_ = false;
  bool has_last_match_ = false;
  bool localized_ = false;
  bool relocalize_ = false;
  bool tf_published_valid_ = false;
  bool dirty_ = false;
  std::int64_t scan_index_ = 0;
  int scans_ = 0;
  int rejected_ = 0;
  double own_map_stamp_ = -1.0;
  std::uint32_t own_map_width_ = 0;
  std::uint32_t own_map_height_ = 0;

  Buffer buffer_;
  std::unique_ptr<TransformListener> listener_;
  TransformBroadcaster broadcaster_;
};

}  // namespace krosc::nodes
