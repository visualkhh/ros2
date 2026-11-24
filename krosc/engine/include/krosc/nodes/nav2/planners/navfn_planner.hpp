#pragma once

#include "krosc/nodes/nav2/planners/global_planner.hpp"

namespace krosc::nodes::nav2 {

class NavfnPlanner final : public NavPlannerPlugin {
 public:
  void configure(Node& node, const std::string& name) override;
  std::vector<std::pair<double, double>> create_plan(const Costmap& costmap, const Pose2& start,
                                                     const Pose2& goal) override;
  [[nodiscard]] std::string type() const override { return "nav2_navfn_planner::NavfnPlanner"; }
  [[nodiscard]] int expanded() const { return expanded_; }

 private:
  Parameter<double> tolerance_;
  Parameter<bool> use_astar_;
  Parameter<bool> allow_unknown_;
  int expanded_ = 0;
};

}  // namespace krosc::nodes::nav2
