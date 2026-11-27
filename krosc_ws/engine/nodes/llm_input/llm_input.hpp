#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "krosc/action.hpp"
#include "krosc/msgs/moveit_msgs.hpp"
#include "krosc/node.hpp"

#include "llm_client.hpp"

namespace krosc_ws {

// 평문 명령을 Ollama /api/chat으로 보내 intent를 뽑고, krosc 도구로 라우팅한다.
// service: llm/command (String -> JSON String)
// topic:   llm/status  (String JSON), 진행 상황/에러
class LlmInput final : public krosc::Node {
 public:
  LlmInput(krosc::Context& context, std::string ollama_url, std::string model,
           double timeout_s, std::string mode = "llm");
  ~LlmInput() override;

  LlmInput(const LlmInput&) = delete;
  LlmInput& operator=(const LlmInput&) = delete;

  // 엔진 정지 시 진행 중인 주행을 즉시 멈춘다.
  void emergency_stop();

 private:
  using MoveClient = krosc::ActionClient<krosc::msgs::moveit::MoveGroupGoal,
                                         krosc::msgs::moveit::MoveGroupFeedback,
                                         krosc::msgs::moveit::MoveGroupResult>;

  struct Request {
    std::uint64_t id = 0;
    std::string text;
  };

  krosc::msgs::String handle_command(const krosc::msgs::String& request);
  void worker_loop();
  void process(const Request& request);
  void publish_status(std::uint64_t id, const std::string& stage,
                      const std::string& extra_json = {});
  void dispatch(std::uint64_t id, const std::string& intent, double meters,
                double degrees, const std::string& direction,
                const std::string& preset, const std::string& melody,
                const std::string& question, const std::string& target,
                const std::string& style);
  std::string play_melody(const std::string& name, std::string& error);
  void start_drive(std::uint64_t id, double meters, double sign);
  void start_turn(std::uint64_t id, const std::string& direction, double degrees);
  void drive_tick();
  void drive_tick_linear();
  void drive_tick_turn();
  void stop_drive();
  void send_arm_preset(std::uint64_t id, const std::string& preset);
  void dance(std::uint64_t id, const std::string& style);
  std::string call_optional(const std::string& service, const std::string& payload,
                            std::string& error);

  std::unique_ptr<OllamaClient> client_;
  std::unique_ptr<MoveClient> move_;
  std::string mode_;
  mutable std::mutex context_mutex_;
  std::string robot_context_;

  std::deque<Request> queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::thread worker_;
  std::atomic<bool> stopping_{false};
  std::atomic<std::uint64_t> next_id_{1};

  mutable std::mutex odom_mutex_;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;
  bool odom_seen_ = false;

  std::atomic<bool> drive_active_{false};
  std::atomic<bool> dance_cancel_{false};
  std::uint64_t drive_request_ = 0;
  int drive_mode_ = 0;  // 0=linear, 1=turn
  double drive_sign_ = 1.0;
  double drive_target_ = 0.0;
  double drive_start_x_ = 0.0;
  double drive_start_y_ = 0.0;
  double drive_started_ = 0.0;
  double drive_last_distance_ = 0.0;
  double drive_progress_time_ = 0.0;
  double drive_timeout_ = 30.0;
  double drive_speed_ = 0.15;
  double turn_start_yaw_ = 0.0;
  double turn_target_ = 0.0;
  double turn_angular_ = 0.0;

  std::mutex move_mutex_;
  std::string move_goal_id_;

  krosc::Timer drive_timer_;
  krosc::Subscription<krosc::msgs::String> context_subscription_;
  krosc::Subscription<krosc::msgs::Odometry> odom_subscription_;
};

}  // namespace krosc_ws
