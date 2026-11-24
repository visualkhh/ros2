#include "krosc/moveit/planners/joint_space_planner.hpp"

namespace krosc::moveit {

PlanningResult JointSpacePlanner::plan(const PlanningRequest& request) {
  return run(request, false);
}

}  // namespace krosc::moveit
