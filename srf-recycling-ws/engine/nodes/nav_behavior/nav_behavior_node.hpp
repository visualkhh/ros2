#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "srf/action.hpp"
#include "srf/msgs/nav2_msgs.hpp"
#include "srf/node.hpp"
#include "srf/tf/tf.hpp"
#include "srf/tf/tf_ros.hpp"

namespace srf_recycling_ws {

// 지도/코스트맵 기반 behavior.
//   nav/hide     : 현재 위치에서 벽에 가려진 free cell을 골라 nav2로 이동
//   nav/approach : 전방 라이다를 보며 벽/물체 앞 0.22m까지 접근
// topic: nav/status (String JSON)
class NavBehaviorNode final : public srf::Node {
 public:
  NavBehaviorNode(srf::Context& context, std::string costmap_topic,
                  std::string scan_topic);
  ~NavBehaviorNode() override;

  NavBehaviorNode(const NavBehaviorNode&) = delete;
  NavBehaviorNode& operator=(const NavBehaviorNode&) = delete;

  void emergency_stop();

 private:
  using NavClient = srf::ActionClient<srf::msgs::nav2::NavigateToPoseGoal,
                                        srf::msgs::nav2::NavigateToPoseFeedback,
                                        srf::msgs::nav2::NavigateToPoseResult>;

  srf::msgs::String handle_hide(const srf::msgs::String& request);
  srf::msgs::String handle_approach(const srf::msgs::String& request);
  srf::msgs::TriggerResponse handle_stop(const srf::msgs::TriggerRequest& request);
  void worker_loop();
  bool run_hide(std::string& error);
  bool run_approach(std::string& error);
  bool hide_goal(double& x, double& y, std::string& error) const;
  bool line_of_sight_blocked(const srf::msgs::OccupancyGrid& grid,
                             double x0, double y0, double x1, double y1) const;
  void publish_status(std::uint64_t id, const std::string& stage,
                      const std::string& extra_json = {});

  std::string costmap_topic_;
  std::string scan_topic_;
  std::unique_ptr<NavClient> nav_;
  std::mutex nav_mutex_;
  std::string nav_goal_id_;

  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic<bool> cancel_{false};
  bool stopping_ = false;
  bool pending_ = false;
  std::uint64_t pending_id_ = 0;
  std::string pending_action_;
  std::atomic<std::uint64_t> next_id_{1};
  std::atomic<std::uint64_t> active_id_{0};

  mutable std::mutex grid_mutex_;
  srf::msgs::OccupancyGrid grid_;
  bool grid_seen_ = false;

  mutable std::mutex scan_mutex_;
  srf::msgs::LaserScan scan_;
  bool scan_seen_ = false;

  srf::Buffer tf_buffer_;
  std::unique_ptr<srf::TransformListener> tf_listener_;

  srf::Subscription<srf::msgs::OccupancyGrid> grid_subscription_;
  srf::Subscription<srf::msgs::LaserScan> scan_subscription_;
};

}  // namespace srf_recycling_ws
