#include <array>

#include "robot/waffle_pi_arm.hpp"

#include "test_framework.hpp"

namespace {

void check_arm(const std::array<double, 5>& expected,
               const std::array<double, 5>& actual, double tolerance) {
  for (std::size_t index = 0; index < expected.size(); ++index) {
    CHECK_NEAR(expected[index], actual[index], tolerance);
  }
}

}  // namespace

KROSC_TEST(waffle_pi_arm_manual_stow_reaches_target) {
  krosc_ws::sim::WafflePiArm robot(false, 7);
  robot.set_manual_arm(true);
  robot.set_arm_target(krosc_ws::sim::WafflePiArm::kStow);
  for (int step = 0; step < 200; ++step) {
    robot.tick(static_cast<double>(step) * 0.01, 0.01);
  }
  const auto status = robot.arm_status();
  CHECK(status.present);
  check_arm(krosc_ws::sim::WafflePiArm::kStow, status.position, 1e-3);

  const auto state = robot.state();
  CHECK_NEAR(krosc_ws::sim::WafflePiArm::kStow[0], state.joints.at("joint1"), 1e-3);
  CHECK_NEAR(krosc_ws::sim::WafflePiArm::kStow[3], state.joints.at("joint4"), 1e-3);
  CHECK_NEAR(krosc_ws::sim::WafflePiArm::kStow[4], state.joints.at("gripper_left_joint"), 1e-3);
}

KROSC_TEST(waffle_pi_arm_controller_target_when_manual_off) {
  krosc_ws::sim::WafflePiArm robot(false, 11);
  robot.set_manual_arm(false);
  robot.set_controller_arm_target(krosc_ws::sim::WafflePiArm::kHome);
  robot.tick(0.01, 0.01);
  check_arm(krosc_ws::sim::WafflePiArm::kHome, robot.arm_status().position, 1e-9);

  robot.set_manual_arm(true);
  robot.set_arm_target(krosc_ws::sim::WafflePiArm::kStow);
  for (int step = 0; step < 200; ++step) {
    robot.tick(0.02 + static_cast<double>(step) * 0.01, 0.01);
  }
  check_arm(krosc_ws::sim::WafflePiArm::kStow, robot.arm_status().position, 1e-3);
}
