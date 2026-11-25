#include "krosc/tf/tf.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace krosc {

namespace {
constexpr double kEpsilon = 1e-9;
}

Buffer::Buffer(double cache_time) : cache_time_(cache_time) {
  if (!(cache_time > 0.0)) {
    throw std::invalid_argument("cache_time은 0보다 커야 한다");
  }
}

void Buffer::set_transform(const std::string& parent, const std::string& child, double stamp,
                           const msgs::Transform& transform, bool is_static) {
  if (parent == child) {
    throw std::invalid_argument("좌표계가 자기 자신의 자식일 수 없다");
  }
  msgs::Transform normalized = transform;
  normalized.rotation = tfmath::normalize(normalized.rotation);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto parent_it = parent_.find(child);
    if (parent_it != parent_.end() && parent_it->second != parent) {
      std::fprintf(stderr, "[krosc/tf] 좌표계 '%s'의 부모가 '%s'에서 '%s'로 바뀌었다 (이전 이력은 버린다)\n",
                   child.c_str(), parent_it->second.c_str(), parent.c_str());
      forget_locked(child);
    }
    // parent의 조상 중에 child가 있으면 순환이다.
    std::optional<std::string> ancestor = parent;
    while (ancestor) {
      if (*ancestor == child) {
        throw ConnectivityException("좌표계 순환이 생긴다: " + parent + " -> " + child);
      }
      auto it = parent_.find(*ancestor);
      ancestor = it == parent_.end() ? std::nullopt : std::optional<std::string>(it->second);
    }
    parent_[child] = parent;
    if (is_static) {
      clear_history_locked(child);
      static_[child] = normalized;
      arrived_.notify_all();
      return;
    }
    static_.erase(child);
    History& history = history_[child];
    if (history.stamps.empty()) {
      history.stamps.push_back(stamp);
      history.samples.push_back(normalized);
      arrived_.notify_all();
      return;
    }
    const double latest = history.stamps.back();
    if (stamp < latest - cache_time_) {
      return;  // 보관 범위보다 오래된 데이터는 무시
    }
    auto it = std::lower_bound(history.stamps.begin(), history.stamps.end(), stamp);
    if (it != history.stamps.end() && std::abs(*it - stamp) <= kEpsilon) {
      history.samples[static_cast<std::size_t>(it - history.stamps.begin())] = normalized;
    } else {
      const auto index = static_cast<std::size_t>(it - history.stamps.begin());
      history.stamps.insert(it, stamp);
      history.samples.insert(history.samples.begin() + static_cast<std::ptrdiff_t>(index), normalized);
    }
    const double cutoff = history.stamps.back() - cache_time_;
    auto drop = std::lower_bound(history.stamps.begin(), history.stamps.end(), cutoff - kEpsilon);
    const auto drop_count = static_cast<std::size_t>(drop - history.stamps.begin());
    if (drop_count > 0) {
      history.stamps.erase(history.stamps.begin(), history.stamps.begin() + static_cast<std::ptrdiff_t>(drop_count));
      history.samples.erase(history.samples.begin(), history.samples.begin() + static_cast<std::ptrdiff_t>(drop_count));
    }
  }
  arrived_.notify_all();
}

void Buffer::set_transform(const msgs::TransformStamped& stamped, bool is_static) {
  set_transform(stamped.header.frame_id, stamped.child_frame_id, stamped.header.stamp, stamped.transform, is_static);
}

msgs::TransformStamped Buffer::lookup_transform(const std::string& target, const std::string& source,
                                                std::optional<double> stamp, double timeout) const {
  if (timeout < 0.0) {
    throw std::invalid_argument("timeout은 0 이상이어야 한다");
  }
  std::unique_lock<std::mutex> lock(mutex_);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                               std::chrono::duration<double>(timeout));
  for (;;) {
    try {
      return lookup_locked(target, source, stamp);
    } catch (const TransformException&) {
      if (timeout <= 0.0 || std::chrono::steady_clock::now() >= deadline) {
        throw;
      }
      arrived_.wait_until(lock, deadline);
    }
  }
}

bool Buffer::can_transform(const std::string& target, const std::string& source, std::optional<double> stamp,
                           double timeout) const {
  try {
    (void)lookup_transform(target, source, stamp, timeout);
    return true;
  } catch (const TransformException&) {
    return false;
  }
}

msgs::Vector3 Buffer::transform_point(const std::string& target, const std::string& source, const msgs::Vector3& point,
                                      std::optional<double> stamp, double timeout) const {
  const msgs::TransformStamped transform = lookup_transform(target, source, stamp, timeout);
  return tfmath::apply(transform.transform, point);
}

msgs::PointStamped Buffer::transform(const msgs::PointStamped& point, const std::string& target_frame,
                                     double timeout) const {
  const std::optional<double> stamp = point.header.stamp == 0.0 ? std::nullopt : std::optional<double>(point.header.stamp);
  const msgs::TransformStamped transform = lookup_transform(target_frame, point.header.frame_id, stamp, timeout);
  const msgs::Vector3 result = tfmath::apply(transform.transform, msgs::Vector3{point.point.x, point.point.y, point.point.z});
  msgs::PointStamped out;
  out.header.stamp = transform.header.stamp;
  out.header.frame_id = target_frame;
  out.point = {result.x, result.y, result.z};
  return out;
}

msgs::PoseStamped Buffer::transform(const msgs::PoseStamped& pose, const std::string& target_frame,
                                    double timeout) const {
  const std::optional<double> stamp = pose.header.stamp == 0.0 ? std::nullopt : std::optional<double>(pose.header.stamp);
  const msgs::TransformStamped transform = lookup_transform(target_frame, pose.header.frame_id, stamp, timeout);
  const msgs::Vector3 position =
      tfmath::apply(transform.transform, msgs::Vector3{pose.pose.position.x, pose.pose.position.y, pose.pose.position.z});
  const msgs::Quaternion orientation =
      tfmath::normalize(tfmath::multiply(transform.transform.rotation, pose.pose.orientation));
  msgs::PoseStamped out;
  out.header.stamp = transform.header.stamp;
  out.header.frame_id = target_frame;
  out.pose.position = {position.x, position.y, position.z};
  out.pose.orientation = orientation;
  return out;
}

std::vector<std::string> Buffer::all_frames() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::unordered_set<std::string> frames;
  for (const auto& [child, parent] : parent_) {
    frames.insert(child);
    frames.insert(parent);
  }
  std::vector<std::string> result(frames.begin(), frames.end());
  std::sort(result.begin(), result.end());
  return result;
}

std::optional<std::string> Buffer::parent_of(const std::string& frame) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = parent_.find(frame);
  if (it == parent_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::string Buffer::all_frames_as_string() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::unordered_set<std::string> frames;
  for (const auto& [child, parent] : parent_) {
    frames.insert(child);
    frames.insert(parent);
  }
  std::vector<std::string> ordered(frames.begin(), frames.end());
  std::sort(ordered.begin(), ordered.end());
  std::string out;
  for (const auto& frame : ordered) {
    auto it = parent_.find(frame);
    out += "Frame " + frame + " exists with parent " + (it == parent_.end() ? "NO_PARENT" : it->second) + ".\n";
  }
  return out;
}

std::string Buffer::all_frames_as_yaml() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> children;
  children.reserve(parent_.size());
  for (const auto& [child, parent] : parent_) {
    (void)parent;
    children.push_back(child);
  }
  std::sort(children.begin(), children.end());
  std::string out;
  char buffer[256];
  for (const auto& child : children) {
    const auto it = history_.find(child);
    double span = 0.0;
    double rate = 0.0;
    double newest = 0.0;
    double oldest = 0.0;
    if (it != history_.end() && !it->second.stamps.empty()) {
      const auto& stamps = it->second.stamps;
      newest = stamps.back();
      oldest = stamps.front();
      span = newest - oldest;
      rate = span > 0.0 ? static_cast<double>(stamps.size() - 1) / span : 0.0;
    }
    out += child + ":\n";
    out += "  parent: '" + parent_.at(child) + "'\n";
    out += std::string("  broadcaster: '") + (static_.count(child) != 0 ? "static" : "tf") + "'\n";
    std::snprintf(buffer, sizeof(buffer), "  rate: %.3f\n", rate);
    out += buffer;
    std::snprintf(buffer, sizeof(buffer), "  most_recent_transform: %.6f\n", newest);
    out += buffer;
    std::snprintf(buffer, sizeof(buffer), "  oldest_transform: %.6f\n", oldest);
    out += buffer;
    std::snprintf(buffer, sizeof(buffer), "  buffer_length: %.3f\n", span);
    out += buffer;
  }
  return out;
}

void Buffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  parent_.clear();
  static_.clear();
  history_.clear();
}

msgs::TransformStamped Buffer::lookup_locked(const std::string& target, const std::string& source,
                                             std::optional<double> stamp) const {
  for (const auto& frame : {target, source}) {
    if (!known_locked(frame)) {
      throw LookupException("좌표계 '" + frame + "'를 모른다 (아직 그 좌표계의 변환을 내보낸 곳이 없다)");
    }
  }
  const std::vector<std::string> target_chain = chain_locked(target);
  const std::vector<std::string> source_chain = chain_locked(source);
  std::unordered_set<std::string> target_set(target_chain.begin(), target_chain.end());
  std::optional<std::string> common;
  for (const auto& frame : source_chain) {
    if (target_set.count(frame) != 0) {
      common = frame;
      break;
    }
  }
  if (!common) {
    throw ConnectivityException("좌표계 '" + target + "'와 '" + source + "'는 변환 트리에서 이어져 있지 않다");
  }
  const auto source_common_it = std::find(source_chain.begin(), source_chain.end(), *common);
  const auto target_common_it = std::find(target_chain.begin(), target_chain.end(), *common);
  const std::vector<std::string> source_edges(source_chain.begin(), source_common_it);
  const std::vector<std::string> target_edges(target_chain.begin(), target_common_it);
  std::vector<std::string> edge_children = source_edges;
  edge_children.insert(edge_children.end(), target_edges.begin(), target_edges.end());
  const double when = stamp.has_value() ? *stamp : latest_common_time_locked(edge_children);
  const msgs::Transform to_common_from_source = fold_locked(source_edges, when);
  const msgs::Transform to_common_from_target = fold_locked(target_edges, when);
  const msgs::Transform result = tfmath::compose(tfmath::inverse(to_common_from_target), to_common_from_source);
  msgs::TransformStamped out;
  out.header.stamp = when;
  out.header.frame_id = target;
  out.child_frame_id = source;
  out.transform = result;
  return out;
}

bool Buffer::known_locked(const std::string& frame) const {
  if (parent_.count(frame) != 0) {
    return true;
  }
  for (const auto& [child, parent] : parent_) {
    (void)child;
    if (parent == frame) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> Buffer::chain_locked(const std::string& frame) const {
  std::vector<std::string> chain{frame};
  while (true) {
    auto it = parent_.find(chain.back());
    if (it == parent_.end()) {
      break;
    }
    chain.push_back(it->second);
  }
  return chain;
}

double Buffer::latest_common_time_locked(const std::vector<std::string>& children) const {
  std::optional<double> latest;
  for (const auto& child : children) {
    if (static_.count(child) != 0) {
      continue;
    }
    auto it = history_.find(child);
    if (it == history_.end() || it->second.stamps.empty()) {
      continue;
    }
    const double last = it->second.stamps.back();
    latest = latest ? std::min(*latest, last) : last;
  }
  return latest.value_or(0.0);
}

msgs::Transform Buffer::fold_locked(const std::vector<std::string>& children, double stamp) const {
  msgs::Transform result = tfmath::kIdentity;
  for (const auto& child : children) {
    result = tfmath::compose(edge_locked(child, stamp), result);
  }
  return result;
}

msgs::Transform Buffer::edge_locked(const std::string& child, double stamp) const {
  auto static_it = static_.find(child);
  if (static_it != static_.end()) {
    return static_it->second;
  }
  auto history_it = history_.find(child);
  if (history_it == history_.end() || history_it->second.stamps.empty()) {
    throw ConnectivityException("좌표계 '" + child + "'의 변환 값이 아직 없다");
  }
  const auto& stamps = history_it->second.stamps;
  const auto& samples = history_it->second.samples;
  if (stamp < stamps.front() - kEpsilon || stamp > stamps.back() + kEpsilon) {
    char message[256];
    std::snprintf(message, sizeof(message), "좌표계 '%s'의 이력은 %.3f~%.3f초인데 %.3f초를 요청했다", child.c_str(),
                  stamps.front(), stamps.back(), stamp);
    throw ExtrapolationException(message);
  }
  auto it = std::lower_bound(stamps.begin(), stamps.end(), stamp - kEpsilon);
  if (it == stamps.begin() || std::abs(*it - stamp) <= kEpsilon) {
    const auto index = std::min(static_cast<std::size_t>(it - stamps.begin()), stamps.size() - 1);
    return samples[index];
  }
  const auto after = static_cast<std::size_t>(it - stamps.begin());
  const std::size_t before = after - 1;
  const double ratio = (stamp - stamps[before]) / (stamps[after] - stamps[before]);
  return tfmath::interpolate(samples[before], samples[after], ratio);
}

void Buffer::clear_history_locked(const std::string& child) { history_.erase(child); }

void Buffer::forget_locked(const std::string& child) {
  static_.erase(child);
  clear_history_locked(child);
}

}  // namespace krosc
