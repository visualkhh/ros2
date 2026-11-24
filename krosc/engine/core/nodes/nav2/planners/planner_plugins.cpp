#include "krosc/nodes/nav2/planners/planner_plugins.hpp"

#include <memory>

#include "krosc/nodes/nav2/planners/navfn_planner.hpp"
#include "krosc/nodes/nav2/planners/smac_planner_2d.hpp"
#include "krosc/nodes/nav2/planners/smac_planner_hybrid.hpp"
#include "krosc/nodes/nav2/planners/theta_star_planner.hpp"

namespace krosc::nodes::nav2 {

PlannerRegistry builtin_planners() {
  PlannerRegistry registry;
  registry.add("nav2_navfn_planner::NavfnPlanner", [] { return std::make_unique<NavfnPlanner>(); });
  registry.add("nav2_theta_star_planner::ThetaStarPlanner", [] { return std::make_unique<ThetaStarPlanner>(); });
  registry.add("nav2_smac_planner::SmacPlanner2D", [] { return std::make_unique<SmacPlanner2D>(); });
  registry.add("nav2_smac_planner::SmacPlannerHybrid", [] { return std::make_unique<SmacPlannerHybrid>(); });
  return registry;
}

}  // namespace krosc::nodes::nav2
