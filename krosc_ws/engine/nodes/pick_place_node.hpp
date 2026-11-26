#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "krosc/action.hpp"
#include "krosc/msgs/moveit_msgs.hpp"
#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/node.hpp"

namespace krosc_ws {

// 시뮬레이터의 graspable 물체를 MoveIt으로 집고 내려놓는 노드.
// 노드가 궤적/그리퍼 시퀀스를 소유하고, 시뮬레이터는 그리퍼가 닫히면 물체를 붙인다.
//
// service: pick/command (String JSON {"action":"pick"|"place","color":"yellow"}) -> accepted
// topic:   pick/status (String JSON), sim_graspable (Path) 구독
class PickPlaceNode final : public krosc::Node {
 public:
  PickPlaceNode(krosc::Context& context, std::string objects_topic);
  ~PickPlaceNode() override;

  PickPlaceNode(const PickPlaceNode&) = delete;
  PickPlaceNode& operator=(const PickPlaceNode&) = delete;

  void emergency_stop();

 private:
  using MoveClient = krosc::ActionClient<krosc::msgs::moveit::MoveGroupGoal,
                                         krosc::msgs::moveit::MoveGroupFeedback,
                                         krosc::msgs::moveit::MoveGroupResult>;
  using NavClient = krosc::ActionClient<krosc::msgs::nav2::NavigateToPoseGoal,
                                        krosc::msgs::nav2::NavigateToPoseFeedback,
                                        krosc::msgs::nav2::NavigateToPoseResult>;

  krosc::msgs::String handle_command(const krosc::msgs::String& request);
  void worker_loop();
  void publish_status(std::uint64_t id, const std::string& stage,
                      const std::string& extra_json = {});
  bool switch_arm(std::string& error);
  bool move_to(std::uint64_t id, const std::string& stage, double x, double y,
               double z, double pitch_deg, double grip_m, std::string& error);
  bool pick(std::uint64_t id, const std::string& color, std::string& error);
  bool place(std::uint64_t id, const std::string& target, std::string& error);
  bool approach(std::uint64_t id, std::string& error);
  bool navigate_to_puck(std::uint64_t id, std::string& error);
  bool object_odom(double& x, double& y, double& z, std::string& error) const;
  bool object_in_base(double& x, double& y, double& z, double& yaw,
                      std::string& error) const;

  std::string objects_topic_;
  std::unique_ptr<MoveClient> move_;
  std::unique_ptr<NavClient> nav_;
  std::mutex move_mutex_;
  std::string move_goal_id_;
  std::mutex nav_mutex_;
  std::string nav_goal_id_;

  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool stopping_ = false;
  bool pending_ = false;
  std::uint64_t pending_id_ = 0;
  std::string pending_action_;
  std::string pending_color_;
  std::string pending_target_;
  std::atomic<std::uint64_t> next_id_{1};

  mutable std::mutex object_mutex_;
  krosc::msgs::Path objects_;
  bool objects_seen_ = false;

  mutable std::mutex odom_mutex_;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;
  bool odom_seen_ = false;

  bool holding_ = false;
  std::atomic<bool> cancel_{false};
  double pick_x_ = 0.0;
  double pick_y_ = 0.0;

  krosc::Subscription<krosc::msgs::Path> objects_subscription_;
  krosc::Subscription<krosc::msgs::Odometry> odom_subscription_;
};

}  // namespace krosc_ws
