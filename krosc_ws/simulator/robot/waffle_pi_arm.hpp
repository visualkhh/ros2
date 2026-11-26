#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include "hardwares/opencr/opencr_device.hpp"
#include "krosc/simulator/robots/turtlebot/turtlebot.hpp"

namespace krosc_ws::sim {

class UrdfRobotVisualizer;

// krosc_ws 실기 모델: TurtleBot3 Waffle Pi 차동 구동 + OpenMANIPULATOR-X 4DOF/gripper.
// 시뮬레이터 전용이며 실기 Robot에는 SimulatorOpenCr/SimulatorLds 하드웨어 스텁으로 연결된다.
class WafflePiArm final : public krosc::simulator::TurtleBot3 {
 public:
  explicit WafflePiArm(bool noisy = false, std::optional<std::uint32_t> seed = std::nullopt,
                       std::string urdf_path = {});
  ~WafflePiArm() override;

  void tick(double time, double dt) override;
  [[nodiscard]] krosc::simulator::RobotState state() const override;

  // 수동 조작(패널)과 MoveIt/컨트롤러 명령의 우선순위를 전환한다.
  void set_manual_arm(bool manual);
  [[nodiscard]] bool manual_arm() const;

  void set_arm_target(const std::array<double, 5>& target);            // 수동
  void set_controller_arm_target(const std::array<double, 5>& target); // SimulatorOpenCr write_arm
  [[nodiscard]] ArmStatus arm_status() const;

  static constexpr std::array<double, 5> kHome{0.5, -1.8, 1.4, 0.8, 0.0};
  static constexpr std::array<double, 5> kStow{0.0, -1.9024, 1.3440, 0.9023, 0.0};

 private:
  static void step_towards(std::array<double, 5>& position, const std::array<double, 5>& target,
                           double dt);
  [[nodiscard]] std::array<double, 5> current_locked() const;

  std::unique_ptr<UrdfRobotVisualizer> visualizer_;
  mutable std::mutex arm_mutex_;
  bool manual_ = true;
  std::array<double, 5> manual_position_{};
  std::array<double, 5> manual_target_{};
  std::array<double, 5> controller_position_{};
  std::array<double, 5> controller_target_{};
};

}  // namespace krosc_ws::sim
