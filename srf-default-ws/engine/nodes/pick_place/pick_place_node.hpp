#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "srf/action.hpp"
#include "srf/msgs/moveit_msgs.hpp"
#include "srf/msgs/nav2_msgs.hpp"
#include "srf/node.hpp"

namespace srf_default_ws {

// 시뮬레이터의 graspable 물체를 MoveIt으로 집고 내려놓는 노드.
// 노드가 궤적/그리퍼 시퀀스를 소유하고, 시뮬레이터는 그리퍼가 닫히면 물체를 붙인다.
//
// service: pick/command (String JSON {"action":"pick"|"place","color":"yellow"}) -> accepted
// topic:   pick/status (String JSON), sim_graspable (Path), grasp_info (String) 구독
class PickPlaceNode final : public srf::Node {
 public:
  PickPlaceNode(srf::Context& context, std::string objects_topic,
                std::string info_topic = "grasp_info");
  ~PickPlaceNode() override;

  PickPlaceNode(const PickPlaceNode&) = delete;
  PickPlaceNode& operator=(const PickPlaceNode&) = delete;

  void emergency_stop();

 private:
  using MoveClient = srf::ActionClient<srf::msgs::moveit::MoveGroupGoal,
                                         srf::msgs::moveit::MoveGroupFeedback,
                                         srf::msgs::moveit::MoveGroupResult>;
  using NavClient = srf::ActionClient<srf::msgs::nav2::NavigateToPoseGoal,
                                        srf::msgs::nav2::NavigateToPoseFeedback,
                                        srf::msgs::nav2::NavigateToPoseResult>;

  srf::msgs::String handle_command(const srf::msgs::String& request);
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
  std::string info_topic_;
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
  srf::msgs::Path objects_;
  bool objects_seen_ = false;
  // pick 명령을 받은 순간의 물체 pose를 고정한다. base가 가까워져 D435에서
  // 퍽이 가려져도 이 좌표로 접근/파지를 계속한다.
  bool have_target_ = false;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_z_ = 0.0;
  double target_width_ = 0.0;
  std::string target_color_;

  // 실기 D435 detector가 보내는 물체 크기. 없으면 기존 4cm 퍽 기본값을 쓴다.
  mutable std::mutex info_mutex_;
  double object_width_ = 0.0;
  double object_height_ = 0.0;
  std::string object_color_;
  double holding_grip_ = 0.0;

  mutable std::mutex odom_mutex_;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;
  bool odom_seen_ = false;

  bool holding_ = false;
  std::atomic<bool> cancel_{false};
  double pick_x_ = 0.0;
  double pick_y_ = 0.0;

  srf::Subscription<srf::msgs::Path> objects_subscription_;
  srf::Subscription<srf::msgs::String> info_subscription_;
  srf::Subscription<srf::msgs::Odometry> odom_subscription_;
};

}  // namespace srf_default_ws
