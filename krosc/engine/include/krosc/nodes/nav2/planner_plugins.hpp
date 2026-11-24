#pragma once

// 편의 umbrella: 플래너 플러그인은 각각 한 파일 세트를 가진다.
#include "krosc/nodes/nav2/global_planner.hpp"
#include "krosc/nodes/nav2/navfn_planner.hpp"
#include "krosc/nodes/nav2/smac_planner_2d.hpp"
#include "krosc/nodes/nav2/smac_planner_hybrid.hpp"
#include "krosc/nodes/nav2/theta_star_planner.hpp"

namespace krosc::nodes::nav2 {

PlannerRegistry builtin_planners();

}  // namespace krosc::nodes::nav2
