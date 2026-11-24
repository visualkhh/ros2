#pragma once

#include "krosc/moveit/kinematics_base.hpp"

namespace krosc::moveit {

// URDF 체인 해석 IK/FK를 KinematicsBase 플러그인으로 감싼다.
class ArmChainKinematics final : public MoveItKinematicsPlugin {
 public:
  [[nodiscard]] KinematicsResult search_position_ik(const KinematicsQuery& query) const override;
  [[nodiscard]] msgs::Vector3 get_position_fk(const std::array<double, 4>& joints) const override;
  [[nodiscard]] std::string type() const override { return "krosc_moveit/ArmChainKinematics"; }
};

}  // namespace krosc::moveit
