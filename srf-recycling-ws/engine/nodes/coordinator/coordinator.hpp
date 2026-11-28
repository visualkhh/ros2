#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "srf/action.hpp"
#include "srf/msgs.hpp"
#include "srf/msgs/moveit_msgs.hpp"
#include "srf/msgs/nav2_msgs.hpp"
#include "srf/node.hpp"

namespace srf_recycling_ws {

// 사람 지시 한 건을 끝까지 해내는 지휘자. 여러 노드를 불러 계획하고 순서대로 실행한다.
//   1) llm/parse 로 할 일(intent)을 받는다 (해석만 LLM, 실행 판단은 여기서 결정적으로).
//   2) intent 에 맞는 스킬을 실행한다:
//      drive/turn/dance(cmd_vel_nav), arm_preset(MoveIt), buzzer, pick/place(pick_place), hide/approach(nav_behavior),
//      vlm_query(vlm), sort = 분리수거 미션:
//        반복 { Nav → 작업 매트 접근점(map_marker) · vlm/select (보류면 멈추고 기록) · vla/execute pick ·
//               Nav → 해당 바구니 접근점 · vla/execute place }  대상이 없어질 때까지
//   3) 진행은 action feedback 으로, 끝은 result 로 (모두 String JSON). 취소하면 하던 동작을 전부 멈춘다.
//
// action:  coordinator/task (goal String 문장 → feedback String JSON → result String JSON)
// service: coordinator/stop (Trigger) — 즉시 정지 (하던 goal 취소 + 주행·pick·nav·vla 정지)
class Coordinator final : public srf::Node {
 public:
  explicit Coordinator(srf::Context& context);
  ~Coordinator() override;

  Coordinator(const Coordinator&) = delete;
  Coordinator& operator=(const Coordinator&) = delete;

  // 엔진 정지·정지 명령: 진행 중인 주행·동작을 즉시 멈춘다.
  void emergency_stop();

 private:
  using TaskServer = srf::ActionServer<srf::msgs::String, srf::msgs::String, srf::msgs::String>;
  using TaskHandle = srf::ServerGoalHandle<srf::msgs::String, srf::msgs::String>;
  using StringClient = srf::ActionClient<srf::msgs::String, srf::msgs::String, srf::msgs::String>;
  using MoveClient = srf::ActionClient<srf::msgs::moveit::MoveGroupGoal, srf::msgs::moveit::MoveGroupFeedback,
                                       srf::msgs::moveit::MoveGroupResult>;
  using NavClient = srf::ActionClient<srf::msgs::nav2::NavigateToPoseGoal, srf::msgs::nav2::NavigateToPoseFeedback,
                                      srf::msgs::nav2::NavigateToPoseResult>;

  struct ActionStatus {
    std::uint64_t id = 0;
    std::uint64_t sequence = 0;
    std::string stage;
    std::string json;
  };
  struct ActionResult {
    std::string stage;  // done|error|cancelled|timeout|safety
    std::string detail;
    std::string json;
  };

  srf::msgs::String execute(TaskHandle& handle);
  void report(const std::string& stage, const std::string& extra_json = {});
  void dispatch(const std::string& intent, double meters, double degrees, const std::string& direction,
                const std::string& preset, const std::string& melody, const std::string& question,
                const std::string& target, const std::string& style);

  // ---- 스킬 ----
  void sort(const std::string& target);
  bool navigate_to_marker(const std::string& marker, std::string& error);
  std::optional<std::string> vla_execute(const std::string& goal_json, std::string& error);
  std::string play_melody(const std::string& name, std::string& error);
  void start_drive(double meters, double sign);
  void start_turn(const std::string& direction, double degrees);
  void wait_drive();
  void drive_tick();
  void drive_tick_linear();
  void drive_tick_turn();
  void stop_drive();
  void send_arm_preset(const std::string& preset);
  void dance(const std::string& style);
  std::string call_optional(const std::string& service, const std::string& payload, std::string& error);
  ActionResult wait_action(const std::string& tool, const std::string& topic, std::uint64_t action_id,
                           double timeout_s);
  std::string safety_stop_reason(const std::string& tool) const;
  [[nodiscard]] bool cancelled() const { return action_cancel_.load(); }

  std::unique_ptr<MoveClient> move_;
  std::unique_ptr<NavClient> nav_;
  std::unique_ptr<StringClient> vla_;
  std::unique_ptr<TaskServer> server_;

  // 지금 실행 중인 goal (feedback 보낼 곳)과 마지막 결과
  std::mutex task_mutex_;
  TaskHandle* current_ = nullptr;
  std::string final_json_;
  std::atomic<std::uint64_t> next_id_{1};
  std::uint64_t task_id_ = 0;

  mutable std::mutex context_mutex_;
  std::string robot_context_;
  std::atomic<bool> action_cancel_{false};
  std::atomic<double> action_front_baseline_{0.0};

  std::mutex action_mutex_;
  std::condition_variable action_cv_;
  std::map<std::string, ActionStatus> action_status_;

  std::mutex markers_mutex_;
  srf::msgs::MapMarkers markers_;
  bool markers_seen_ = false;

  mutable std::mutex odom_mutex_;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;
  bool odom_seen_ = false;

  std::atomic<bool> drive_active_{false};
  std::atomic<bool> dance_cancel_{false};
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
  std::string nav_goal_id_;
  std::string vla_goal_id_;

  srf::Timer drive_timer_;
  srf::Subscription<srf::msgs::String> context_subscription_;
  srf::Subscription<srf::msgs::Odometry> odom_subscription_;
  srf::Subscription<srf::msgs::MapMarkers> markers_subscription_;
  srf::Subscription<srf::msgs::String> pick_status_subscription_;
  srf::Subscription<srf::msgs::String> nav_status_subscription_;
};

}  // namespace srf_recycling_ws
