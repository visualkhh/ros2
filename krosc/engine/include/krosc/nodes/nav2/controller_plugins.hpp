#pragma once

// 편의 umbrella: 컨트롤러 플러그인은 각각 한 파일 세트를 가진다.
#include "krosc/nodes/nav2/controller.hpp"
#include "krosc/nodes/nav2/dwb_local_planner.hpp"
#include "krosc/nodes/nav2/mppi_controller.hpp"
#include "krosc/nodes/nav2/regulated_pure_pursuit_controller.hpp"

namespace krosc::nodes::nav2 {

ControllerRegistry builtin_controllers();

}  // namespace krosc::nodes::nav2
