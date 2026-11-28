#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <string>

#include "robot/arm_collision.hpp"
#include "robot/waffle_pi_arm.hpp"
#include "test_framework.hpp"

namespace {

using srf_recycling_ws::sim::ArmCollision;

std::map<std::string, double> joints(const std::array<double, 5>& q) {
  return {{"joint1", q[0]}, {"joint2", q[1]}, {"joint3", q[2]}, {"joint4", q[3]},
          {"gripper_left_joint", q[4]}, {"gripper_right_joint", q[4]}};
}

std::string contacts_at(const ArmCollision& collision, const std::array<double, 5>& q) {
  return srf_recycling_ws::sim::to_string(collision.contacts(joints(q)));
}

}  // namespace

SRF_TEST(arm_collision_srdf_disables_adjacent_pairs) {
  const auto pairs = ArmCollision::parse_disabled_pairs(
      R"(<disable_collisions link1="link1" link2="base_link" reason="Adjacent"/>
         <disable_collisions link1="camera_link" link2="d435_mount" reason="Adjacent"/>)");
  CHECK_EQ(pairs.size(), std::size_t{2});
  CHECK((pairs.count({"base_link", "link1"}) == 1));
  CHECK((pairs.count({"camera_link", "d435_mount"}) == 1));
}

SRF_TEST(arm_collision_working_poses_are_free) {
  const ArmCollision collision(SRF_RECYCLING_WS_URDF_PATH, SRF_RECYCLING_WS_SRDF_PATH);
  // 0 자세, SRDF home, 위로 든 자세, 매트 집기 자세들 (데이터 생성기 프리셋)
  for (const auto& q : std::array<std::array<double, 5>, 6>{{{0, 0, 0, 0, 0},
                                                             {0, -1, 0.7, 0.3, 0},
                                                             {0.0, -1.0, 0.3, 0.7, 0.015},
                                                             {0.0, 1.3, -1.0, 0.7, 0.019},
                                                             {0.0, 0.9, -0.6, 1.0, 0.005},
                                                             {0.6, 0.4, -0.2, 0.9, 0.012}}}) {
    CHECK_EQ(contacts_at(collision, q), std::string{});
  }
}

SRF_TEST(arm_collision_detects_camera_body_and_floor) {
  const ArmCollision collision(SRF_RECYCLING_WS_URDF_PATH, SRF_RECYCLING_WS_SRDF_PATH);
  // 앞으로 숙여 그리퍼가 RealSense에 닿음
  CHECK((contacts_at(collision, {0.0, 0.85, 0.15, -0.30, 0.0}).find("camera_link↔link5") != std::string::npos));
  // 더 숙이면 D435 마운트까지
  CHECK((contacts_at(collision, {0.0, 1.0, 0.3, -0.7, 0.0}).find("d435_mount↔link5") != std::string::npos));
  // 위로 꺾어 link4 가 카메라 위로 내려옴
  CHECK((contacts_at(collision, {0.0, 1.45, -0.75, -1.3, 0.0}).find("camera_link↔link4") != std::string::npos));
  // 옆으로 내려 그리퍼가 차체 옆면·바닥에 닿음
  const std::string side = contacts_at(collision, {1.57, 0.7, 0.6, 0.3, 0.0});
  CHECK((side.find("base_link↔gripper_left_link") != std::string::npos));
  CHECK((side.find("gripper_left_link↔floor") != std::string::npos));
}

SRF_TEST(arm_collision_simulated_arm_stops_before_camera) {
  srf_recycling_ws::sim::WafflePiArm robot(false, 3, SRF_RECYCLING_WS_URDF_PATH, SRF_RECYCLING_WS_SRDF_PATH);
  const ArmCollision collision(SRF_RECYCLING_WS_URDF_PATH, SRF_RECYCLING_WS_SRDF_PATH);
  const std::array<double, 5> into_camera{0.0, 1.0, 0.3, -0.7, 0.0};  // 카메라·마운트 깊숙이
  robot.set_arm_target(into_camera);
  for (int step = 0; step < 300; ++step) {
    robot.tick(step * 0.01, 0.01);
  }
  const auto position = robot.arm_status().position;
  CHECK_EQ(contacts_at(collision, position), std::string{});  // 뚫고 들어가지 않는다
  CHECK((robot.arm_blocked_by().find("camera_link") != std::string::npos));
  double gap = 0.0;  // 목표(닿는 자세)까지 못 간다
  for (std::size_t i = 0; i < 5; ++i) {
    gap = std::max(gap, std::abs(position[i] - into_camera[i]));
  }
  CHECK(gap > 0.01);

  // 컨트롤러(MoveIt/궤적) 경로도 같다: 한 번에 큰 목표를 줘도 중간에서 멈춘다
  robot.set_manual_arm(false);
  robot.set_controller_arm_target({0.0, 1.45, -0.75, -1.3, 0.0});
  robot.tick(3.01, 0.01);
  CHECK_EQ(contacts_at(collision, robot.arm_status().position), std::string{});
  CHECK((robot.arm_blocked_by().find("camera_link") != std::string::npos));
}

