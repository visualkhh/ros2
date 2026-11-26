#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "krosc/action.hpp"
#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/tf/tf.hpp"
#include "krosc/tf/tf_ros.hpp"

namespace krosc_ws {

// 지도/코스트맵 기반 behavior.
//   nav/hide     : 현재 위치에서 벽에 가려진 free cell을 골라 nav2로 이동
//   nav/approach : 전방 라이다를 보며 벽/물체 앞 0.22m까지 접근
// topic: nav/status (String JSON)
class NavBehaviorNode final : public krosc::Node {
 public:
  NavBehaviorNode(krosc::Context& context, std::string costmap_topic,
                  std::string scan_topic);
  ~NavBehaviorNode() override;

  NavBehaviorNode(const NavBehaviorNode&) = delete;
  NavBehaviorNode& operator=(const NavBehaviorNode&) = delete;

  void emergency_stop();

 private:
  using NavClient = krosc::ActionClient<krosc::msgs::nav2::NavigateToPoseGoal,
                                        krosc::msgs::nav2::NavigateToPoseFeedback,
                                        krosc::msgs::nav2::NavigateToPoseResult>;

  krosc::msgs::String handle_hide(const krosc::msgs::String& request);
  krosc::msgs::String handle_approach(const krosc::msgs::String& request);
  krosc::msgs::TriggerResponse handle_stop(const krosc::msgs::TriggerRequest& request);
  void worker_loop();
  bool run_hide(std::string& error);
  bool run_approach(std::string& error);
  bool hide_goal(double& x, double& y, std::string& error) const;
  bool line_of_sight_blocked(const krosc::msgs::OccupancyGrid& grid,
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
  krosc::msgs::OccupancyGrid grid_;
  bool grid_seen_ = false;

  mutable std::mutex scan_mutex_;
  krosc::msgs::LaserScan scan_;
  bool scan_seen_ = false;

  krosc::Buffer tf_buffer_;
  std::unique_ptr<krosc::TransformListener> tf_listener_;

  krosc::Subscription<krosc::msgs::OccupancyGrid> grid_subscription_;
  krosc::Subscription<krosc::msgs::LaserScan> scan_subscription_;
};

}  // namespace krosc_ws
