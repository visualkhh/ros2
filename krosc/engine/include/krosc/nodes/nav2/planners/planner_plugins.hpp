#pragma once

// 편의 umbrella: 플래너 플러그인은 각각 한 파일 세트를 가진다.
#include "krosc/nodes/nav2/planners/global_planner.hpp"
#include "krosc/nodes/nav2/planners/navfn_planner.hpp"
#include "krosc/nodes/nav2/planners/smac_planner_2d.hpp"
#include "krosc/nodes/nav2/planners/smac_planner_hybrid.hpp"
#include "krosc/nodes/nav2/planners/theta_star_planner.hpp"

namespace krosc::nodes::nav2 {

PlannerRegistry builtin_planners();

}  // namespace krosc::nodes::nav2
