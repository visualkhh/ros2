#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "srf/action.hpp"
#include "srf/manipulation/arm_chain.hpp"
#include "srf/msgs.hpp"
#include "srf/node.hpp"
#include "srf/tf/tf.hpp"
#include "srf/tf/tf_ros.hpp"

namespace srf_recycling_ws {

// VLA: 영상 + 지시 + 로봇 상태 → 로봇이 움직일 수 있는 모든 것 (바퀴 v·ω, 팔 j1-4, 그리퍼, 부저).
//   4 Hz 로 관측(ml/vla/data.py 와 같은 29개 상태 + RGB + 깊이)을 sidecar(ml/vla/serve.py /act)에 보내
//   앞으로 8 스텝 행동 청크를 받아 replan_steps 만큼 실행하고 다시 묻는다.
//   대상 3D 는 goal 시작 때 odom 에 고정해 로봇이 움직여도 같은 물체를 가리키고, 바구니는 map_marker + TF.
//   어댑터(모델 밖): 속도·관절 한계로 자르고(잘린 값은 기록), 팔은 arm_command_controller 로 직접.
//   끝: 행동 청크가 멈춘 상태(바퀴 0, 관절 변화 없음)가 연속 2번 → 성공으로 보고. 시간 초과·취소는 실패.
//
// action:  vla/execute (goal String JSON {"kind":"pick|place","instruction","target_xyz":[x,y,z] (base_link),
//                                           "bin":"bin_can"} → feedback String JSON → result String JSON {"ok",...})
// service: vla/stop (Trigger)
class VlaNode final : public srf::Node {
 public:
  struct Options {
    std::string url = "http://127.0.0.1:8765/act";
    std::string color_topic = "camera/color/image_raw";
    std::string depth_topic = "camera/depth/image_rect_raw";
    double period_s = 0.25;
    double timeout_s = 120.0;      // goal 하나 최대 시간
    double request_timeout_s = 20.0;
    std::size_t replan_steps = 4;  // 청크에서 실행할 스텝 수 (나머지는 버리고 새 관측으로)
    std::string urdf;              // 엔드이펙터 FK
  };

  VlaNode(srf::Context& context, Options options);
  ~VlaNode() override;

  VlaNode(const VlaNode&) = delete;
  VlaNode& operator=(const VlaNode&) = delete;

  void emergency_stop();

 private:
  using Server = srf::ActionServer<srf::msgs::String, srf::msgs::String, srf::msgs::String>;
  using Handle = srf::ServerGoalHandle<srf::msgs::String, srf::msgs::String>;
  using Action = std::array<double, 8>;  // v, w, j1, j2, j3, j4, gripper, buzzer

  srf::msgs::String execute(Handle& handle);
  std::optional<std::vector<Action>> request(const std::string& instruction, const std::string& state_json,
                                             std::string& error);
  std::optional<std::string> state_json(const std::optional<srf::msgs::Point>& target_odom, const std::string& bin,
                                        std::string& error);
  void apply(const Action& action, bool& buzzer_on, std::string& clipped);
  void stop_motion();
  bool switch_arm(bool manual, std::string& error);

  Options options_;
  std::optional<srf::manipulation::ArmChain> chain_;
  std::unique_ptr<Server> server_;
  srf::Buffer tf_buffer_;
  std::unique_ptr<srf::TransformListener> tf_listener_;
  std::atomic<bool> stop_{false};

  std::mutex mutex_;
  srf::msgs::Image color_;
  srf::msgs::Image depth_;
  srf::msgs::JointState joints_;
  double odom_v_ = 0.0;
  double odom_w_ = 0.0;
  double front_ = 3.5;
  srf::msgs::MapMarkers markers_;
  bool color_seen_ = false, depth_seen_ = false, joints_seen_ = false, odom_seen_ = false;

  srf::Subscription<srf::msgs::Image> color_subscription_;
  srf::Subscription<srf::msgs::Image> depth_subscription_;
  srf::Subscription<srf::msgs::JointState> joints_subscription_;
  srf::Subscription<srf::msgs::Odometry> odom_subscription_;
  srf::Subscription<srf::msgs::LaserScan> scan_subscription_;
  srf::Subscription<srf::msgs::MapMarkers> markers_subscription_;
};

}  // namespace srf_recycling_ws
