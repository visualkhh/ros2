#include <memory>

#include "krosc/nodes/moveit/kinematics/arm_chain_kinematics.hpp"
#include "krosc/nodes/moveit/kinematics/kinematics_base.hpp"

namespace krosc::moveit {

KinematicsRegistry builtin_kinematics() {
  KinematicsRegistry registry;
  registry.add("krosc_moveit/ArmChainKinematics",
               [] { return std::make_unique<ArmChainKinematics>(); });
  return registry;
}

}  // namespace krosc::moveit
