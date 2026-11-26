#include "waffle_pi_arm.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <utility>

#include "krosc/tf/math.hpp"

#include "robot/urdf_robot_visualizer.hpp"

namespace krosc_ws::sim {

namespace msgs = krosc::msgs;
namespace tfmath = krosc::tfmath;

WafflePiArm::WafflePiArm(bool noisy, std::optional<std::uint32_t> seed,
                         std::string urdf_path)
    : TurtleBot3(noisy, seed) {
  if (urdf_path.empty()) {
    return;
  }
  std::ifstream stream(urdf_path);
  if (!stream) {
    std::fprintf(stderr, "[waffle_pi_arm] URDF를 읽지 못했다: %s\n", urdf_path.c_str());
    return;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  try {
    visualizer_ = std::make_unique<UrdfRobotVisualizer>(buffer.str());
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[waffle_pi_arm] URDF visual 파싱 실패: %s\n", error.what());
  }
}

void WafflePiArm::step_towards(std::array<double, 5>& position,
                               const std::array<double, 5>& target, double dt) {
  for (std::size_t index = 0; index < position.size(); ++index) {
    const double max_step = (index == 4 ? 0.03 : 1.2) * dt;
    const double delta = std::clamp(target[index] - position[index], -max_step, max_step);
    position[index] += delta;
  }
}

void WafflePiArm::tick(double time, double dt) {
  TurtleBot3::tick(time, dt);
  if (dt <= 0.0) {
    return;
  }
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (manual_) {
    step_towards(manual_position_, manual_target_, dt);
  } else {
    controller_position_ = controller_target_;
  }
}

void WafflePiArm::set_manual_arm(bool manual) {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (manual_ == manual) {
    return;
  }
  manual_ = manual;
  if (manual_) {
    manual_position_ = controller_position_;
    manual_target_ = manual_position_;
  } else {
    controller_position_ = manual_position_;
    controller_target_ = controller_position_;
  }
}

bool WafflePiArm::manual_arm() const {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  return manual_;
}

void WafflePiArm::set_arm_target(const std::array<double, 5>& target) {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  manual_ = true;
  manual_target_ = target;
}

void WafflePiArm::set_controller_arm_target(const std::array<double, 5>& target) {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  if (!manual_) {
    controller_target_ = target;
  }
}

std::array<double, 5> WafflePiArm::current_locked() const {
  return manual_ ? manual_position_ : controller_position_;
}

ArmStatus WafflePiArm::arm_status() const {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  ArmStatus status;
  status.position = current_locked();
  status.present = true;
  return status;
}

krosc::simulator::RobotState WafflePiArm::state() const {
  auto result = TurtleBot3::state();
  const ArmStatus arm = arm_status();
  result.joints["joint1"] = arm.position[0];
  result.joints["joint2"] = arm.position[1];
  result.joints["joint3"] = arm.position[2];
  result.joints["joint4"] = arm.position[3];
  result.joints["gripper_left_joint"] = arm.position[4];
  result.joints["gripper_right_joint"] = arm.position[4];
  if (visualizer_) {
    result.visuals = visualizer_->visuals(pose, result.joints);
  }
  return result;
}

WafflePiArm::~WafflePiArm() = default;

}  // namespace krosc_ws::sim
