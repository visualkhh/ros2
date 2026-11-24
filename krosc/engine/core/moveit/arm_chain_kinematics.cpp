#include "krosc/moveit/arm_chain_kinematics.hpp"

namespace krosc::moveit {

KinematicsResult ArmChainKinematics::search_position_ik(const KinematicsQuery& query) const {
  KinematicsResult result;
  if (chain_ == nullptr || !chain_->valid()) {
    result.error = "URDF 체인이 없다";
    return result;
  }
  result.success = chain_->ik(query.target, query.pitch_down, query.seed, result.solution,
                              result.error, query.fast);
  return result;
}

msgs::Vector3 ArmChainKinematics::get_position_fk(const std::array<double, 4>& joints) const {
  return chain_ == nullptr ? msgs::Vector3{} : chain_->tcp(joints);
}

}  // namespace krosc::moveit
