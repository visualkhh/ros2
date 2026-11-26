#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "krosc/node.hpp"

namespace krosc_ws {

// VLA 정책 sidecar(HTTP)를 호출해 4Hz로 청크를 실행하는 노드.
// 주기/안전/컨트롤러 전환은 이 노드가 소유한다. sidecar는 model.forward만 한다.
//
// service: vla/command (String JSON {"target":"red block"}) -> accepted JSON
//          vla/stop    (Trigger)
// topic:   vla/status  (String JSON)
class VlaNode final : public krosc::Node {
 public:
  VlaNode(krosc::Context& context, std::string url, std::string camera_topic,
          double period_s, double timeout_s);
  ~VlaNode() override;

  VlaNode(const VlaNode&) = delete;
  VlaNode& operator=(const VlaNode&) = delete;

  void emergency_stop();

 private:
  struct Action {
    std::array<double, 7> value{};
  };

  krosc::msgs::String handle_command(const krosc::msgs::String& request);
  krosc::msgs::TriggerResponse handle_stop(const krosc::msgs::TriggerRequest& request);
  void worker_loop();
  void tick();
  void apply_action(const Action& action);
  void stop_motion();
  void publish_status(std::uint64_t id, const std::string& stage,
                      const std::string& extra_json = {});
  bool request_plan(std::uint64_t id, const std::string& target,
                    std::vector<Action>& actions, std::string& instruction,
                    std::string& error);
  bool switch_arm(bool manual, std::string& error);

  std::string url_;
  std::string camera_topic_;
  double period_s_ = 0.25;
  double timeout_s_ = 180.0;

  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool stopping_ = false;
  bool cancel_ = false;
  bool pending_ = false;
  std::uint64_t pending_id_ = 0;
  std::uint64_t next_id_ = 1;
  std::string pending_target_;

  std::deque<Action> actions_;
  bool plan_active_ = false;
  std::uint64_t plan_id_ = 0;
  std::size_t plan_index_ = 0;
  std::size_t plan_total_ = 0;
  std::string plan_instruction_;

  std::mutex image_mutex_;
  krosc::msgs::Image image_;
  bool image_seen_ = false;

  std::mutex joints_mutex_;
  std::array<double, 5> joints_{};
  bool joints_seen_ = false;

  std::mutex odom_mutex_;
  double linear_ = 0.0;
  double yaw_rate_ = 0.0;
  bool odom_seen_ = false;

  krosc::Timer timer_;
  krosc::Subscription<krosc::msgs::Image> camera_subscription_;
  krosc::Subscription<krosc::msgs::JointState> joint_subscription_;
  krosc::Subscription<krosc::msgs::Odometry> odom_subscription_;
};

}  // namespace krosc_ws
