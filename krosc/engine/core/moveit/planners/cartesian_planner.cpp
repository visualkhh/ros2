#include "krosc/moveit/planners/cartesian_planner.hpp"

namespace krosc::moveit {

PlanningResult CartesianPlanner::plan(const PlanningRequest& request) {
  return run(request, request.cartesian);
}

}  // namespace krosc::moveit
