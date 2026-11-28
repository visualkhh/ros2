#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "render/scene_renderer.hpp"
#include "srf/urdf.hpp"

namespace srf_recycling_ws::sim {

// 로봇 자기 충돌 + 팔-바닥 충돌 검사 (MoveIt 충돌 검사의 축소판).
//   - 형상: URDF <collision> 그대로 (STL 메시는 실제 부품 삼각형, 상자·원기둥·구는 메시로 만든다).
//     메시마다 경계 상자 트리(BVH)를 한 번 만들고, 두 트리를 같이 내려가 삼각형끼리 분리축으로 검사한다.
//   - 검사하지 않는 쌍: SRDF <disable_collisions> (MoveIt Setup Assistant 결과: Adjacent/Never/User).
//   - 팔 링크(회전·직선 관절 아래 링크)만 움직이므로 팔이 낀 쌍만 본다. 팔 링크는 바닥(z<0)도 본다.
class ArmCollision {
 public:
  ArmCollision(const std::string& urdf_path, const std::string& srdf_path);
  ~ArmCollision();

  struct Contact {
    std::string a;
    std::string b;  // "floor" = 바닥
  };
  // joints: 관절 이름 → 값 (WafflePiArm::joint_positions 형식). 자기 충돌이라 차체 자세와 무관하다.
  [[nodiscard]] std::vector<Contact> contacts(const std::map<std::string, double>& joints) const;
  [[nodiscard]] std::size_t pair_count() const { return pairs_.size(); }

  static std::set<std::pair<std::string, std::string>> parse_disabled_pairs(const std::string& srdf_text);

  struct Bvh;

 private:
  struct Body {
    std::string link;
    render::Pose offset;  // link_from_mesh
    std::shared_ptr<const Bvh> bvh;
  };

  srf::UrdfRobot robot_;
  std::vector<Body> bodies_;
  std::set<std::string> arm_links_;
  std::vector<std::pair<std::size_t, std::size_t>> pairs_;  // 검사할 형상 쌍
};

std::string to_string(const std::vector<ArmCollision::Contact>& contacts);

}  // namespace srf_recycling_ws::sim
