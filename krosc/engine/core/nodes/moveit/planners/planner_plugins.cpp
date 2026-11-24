#include <memory>

#include "krosc/nodes/moveit/planners/cartesian_planner.hpp"
#include "krosc/nodes/moveit/planners/joint_space_planner.hpp"
#include "krosc/nodes/moveit/planners/planner_manager.hpp"

namespace krosc::moveit {

PlannerRegistry builtin_planners() {
  PlannerRegistry registry;
  registry.add("krosc_moveit/CartesianPlanner", [] { return std::make_unique<CartesianPlanner>(); });
  registry.add("krosc_moveit/JointSpacePlanner", [] { return std::make_unique<JointSpacePlanner>(); });
  return registry;
}

}  // namespace krosc::moveit
