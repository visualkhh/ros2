#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "krosc/math.hpp"
#include "krosc/msgs.hpp"

namespace krosc {

class TransformException : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
class LookupException : public TransformException {
 public:
  using TransformException::TransformException;
};
class ConnectivityException : public TransformException {
 public:
  using TransformException::TransformException;
};
class ExtrapolationException : public TransformException {
 public:
  using TransformException::TransformException;
};

// 좌표계 트리와 변환 이력 (ROS tf2_ros::Buffer).
// - 움직이는 변환은 시각별 이력을 cache_time초 보관하고 조회할 때 앞뒤를 보간한다.
// - 정적 변환은 하나만 들고 있고 시각과 무관하다.
// - lookup_transform(target, source, time): source의 점을 target으로 옮기는 변환.
//   time이 없으면 움직이는 변환들이 가진 가장 최근 공통 시각을 쓴다.
class Buffer {
 public:
  explicit Buffer(double cache_time = 10.0);

  void set_transform(const std::string& parent, const std::string& child, double stamp, const msgs::Transform& transform,
                     bool is_static = false);
  void set_transform(const msgs::TransformStamped& stamped, bool is_static = false);

  [[nodiscard]] msgs::TransformStamped lookup_transform(const std::string& target, const std::string& source,
                                                        std::optional<double> stamp = std::nullopt,
                                                        double timeout = 0.0) const;
  [[nodiscard]] bool can_transform(const std::string& target, const std::string& source,
                                   std::optional<double> stamp = std::nullopt, double timeout = 0.0) const;
  [[nodiscard]] msgs::Vector3 transform_point(const std::string& target, const std::string& source,
                                              const msgs::Vector3& point, std::optional<double> stamp = std::nullopt,
                                              double timeout = 0.0) const;
  [[nodiscard]] msgs::PointStamped transform(const msgs::PointStamped& point, const std::string& target_frame,
                                             double timeout = 0.0) const;
  [[nodiscard]] msgs::PoseStamped transform(const msgs::PoseStamped& pose, const std::string& target_frame,
                                            double timeout = 0.0) const;

  [[nodiscard]] std::vector<std::string> all_frames() const;
  [[nodiscard]] std::optional<std::string> parent_of(const std::string& frame) const;
  [[nodiscard]] std::string all_frames_as_string() const;
  [[nodiscard]] std::string all_frames_as_yaml() const;
  void clear();

  [[nodiscard]] double cache_time() const { return cache_time_; }

 private:
  struct History {
    std::vector<double> stamps;
    std::vector<msgs::Transform> samples;
  };

  [[nodiscard]] msgs::TransformStamped lookup_locked(const std::string& target, const std::string& source,
                                                     std::optional<double> stamp) const;
  [[nodiscard]] bool known_locked(const std::string& frame) const;
  [[nodiscard]] std::vector<std::string> chain_locked(const std::string& frame) const;
  [[nodiscard]] double latest_common_time_locked(const std::vector<std::string>& children) const;
  [[nodiscard]] msgs::Transform fold_locked(const std::vector<std::string>& children, double stamp) const;
  [[nodiscard]] msgs::Transform edge_locked(const std::string& child, double stamp) const;
  void clear_history_locked(const std::string& child);
  void forget_locked(const std::string& child);

  double cache_time_;
  mutable std::mutex mutex_;
  mutable std::condition_variable arrived_;
  std::unordered_map<std::string, std::string> parent_;  // 자식 -> 부모
  std::unordered_map<std::string, msgs::Transform> static_;  // 자식 -> 정적 변환
  std::unordered_map<std::string, History> history_;
};

}  // namespace krosc
