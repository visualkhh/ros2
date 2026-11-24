#pragma once

#include "krosc/nodes/nav2/global_planner.hpp"

namespace krosc::nodes::nav2 {

class ThetaStarPlanner final : public NavPlannerPlugin {
 public:
  void configure(Node& node, const std::string& name) override;
  std::vector<std::pair<double, double>> create_plan(const Costmap& costmap, const Pose2& start,
                                                     const Pose2& goal) override;
  [[nodiscard]] std::string type() const override { return "nav2_theta_star_planner::ThetaStarPlanner"; }
  [[nodiscard]] int expanded() const { return expanded_; }

 private:
  Parameter<double> w_euc_cost_;
  Parameter<double> w_traversal_cost_;
  Parameter<bool> allow_unknown_;
  Parameter<double> tolerance_;
  int expanded_ = 0;
};

}  // namespace krosc::nodes::nav2
