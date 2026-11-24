#pragma once

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/urdf.hpp"

// MoveIt-lite의 핵심: URDF 체인에서 팔 링크/관절을 뽑아 FK/IK를 계산하고,
// 관절공간/Cartesian 궤적을 시간 파라미터화한다. ROS나 MoveIt 패키지에 의존하지 않는다.
namespace krosc::manipulation {

struct ArmChainSegment {
  std::string joint;
  std::string child_link;
  std::string type;
  msgs::Transform origin;  // 부모 링크 기준 관절 원점
  msgs::Vector3 axis{1.0, 0.0, 0.0};
  bool movable = false;
  bool prismatic = false;
  int joint_index = -1;
};

struct ArmChain {
  std::string base_link;
  std::string tip_link;
  std::vector<std::string> joint_names;
  std::array<double, 4> lower{{-3.141592653589793, -1.5, -1.5, -1.7}};
  std::array<double, 4> upper{{3.141592653589793, 1.5, 1.4, 1.97}};
  std::vector<ArmChainSegment> segments;

  // IK 초기해(q1)를 계산하기 위한 첫 movable 관절의 부모 링크/원점.
  msgs::Transform base_to_proximal{};
  msgs::Transform proximal_joint_origin{};

  [[nodiscard]] static std::optional<ArmChain> from_urdf(const UrdfRobot& robot,
                                                         const std::string& base_link,
                                                         const std::string& tip_link,
                                                         const std::vector<std::string>& joint_names,
                                                         std::string& error);

  [[nodiscard]] bool valid() const { return joint_names.size() == 4 && segments.size() >= 2; }

  // q는 joint_names 순서(rad). 반환값은 base_link 기준 tip_link 변환.
  [[nodiscard]] msgs::Transform fk(const std::array<double, 4>& q) const;
  [[nodiscard]] std::vector<std::pair<std::string, msgs::Transform>> link_transforms(
      const std::array<double, 4>& q) const;
  [[nodiscard]] msgs::Vector3 tcp(const std::array<double, 4>& q) const;
  // 접근축(+x)이 아래를 보는 각도 [rad]. 0=수평, +면 아래.
  [[nodiscard]] double pitch_down(const std::array<double, 4>& q) const;
  [[nodiscard]] double tool_yaw(const std::array<double, 4>& q) const;

  // target은 base_link 기준 TCP 위치, pitch_down은 접근축 하향 각도(rad).
  // seed에서 시작해 관절한계를 지키는 해를 찾는다.
  // fast=true면 seed에서 가까운 해가 나오는 즉시 반환한다 (드래그 preview용).
  [[nodiscard]] bool ik(const msgs::Vector3& target, double pitch_down,
                        const std::array<double, 4>& seed, std::array<double, 4>& solution,
                        std::string& error, bool fast = false) const;
};

struct TrajectoryPoint {
  std::array<double, 4> position{};
  double time = 0.0;
};

struct Trajectory {
  std::vector<TrajectoryPoint> points;
  std::vector<msgs::Vector3> tcp_path;
  double duration = 0.0;
  bool cartesian = false;
  bool fallback = false;
  std::string note;
};

struct PlanOptions {
  double max_joint_speed = 0.6;    // rad/s
  double max_joint_accel = 1.5;    // rad/s^2
  double sample_time = 0.05;       // s (20 Hz)
  bool cartesian = true;           // TCP 직선 보간, 실패하면 joint-space로 자동 전환
  double cartesian_speed = 0.08;   // m/s
  double cartesian_accel = 0.25;   // m/s^2
  double cartesian_step = 0.004;   // m
};

// start/goal은 joint_names 순서(rad). Cartesian을 요청했는데 경로 IK가 실패하면
// joint-space 계획으로 자동 대체하고 fallback=true를 채운다.
[[nodiscard]] std::optional<Trajectory> plan_trajectory(const ArmChain& chain,
                                                        const std::array<double, 4>& start,
                                                        const std::array<double, 4>& goal,
                                                        const PlanOptions& options,
                                                        std::string& error);

}  // namespace krosc::manipulation
