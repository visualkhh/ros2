#pragma once

#include "krosc/nodes/moveit/planners/planner_manager.hpp"

namespace krosc::moveit {

// TCP 직선(Cartesian) 궤적을 계획한다. 실패하면 joint-space로 자동 대체한다.
class CartesianPlanner final : public MoveItPlannerPlugin {
 public:
  [[nodiscard]] PlanningResult plan(const PlanningRequest& request) override;
  [[nodiscard]] std::string type() const override { return "krosc_moveit/CartesianPlanner"; }
};

}  // namespace krosc::moveit
