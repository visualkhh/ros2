#pragma once

#include "krosc/moveit/planners/planner_manager.hpp"

namespace krosc::moveit {

// 관절 공간에서 시작/목표를 보간한다.
class JointSpacePlanner final : public MoveItPlannerPlugin {
 public:
  [[nodiscard]] PlanningResult plan(const PlanningRequest& request) override;
  [[nodiscard]] std::string type() const override { return "krosc_moveit/JointSpacePlanner"; }
};

}  // namespace krosc::moveit
