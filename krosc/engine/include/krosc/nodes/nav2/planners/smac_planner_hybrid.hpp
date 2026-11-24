#pragma once

#include "krosc/nodes/nav2/planners/global_planner.hpp"

namespace krosc::nodes::nav2 {

class SmacPlannerHybrid final : public NavPlannerPlugin {
 public:
  void configure(Node& node, const std::string& name) override;
  std::vector<std::pair<double, double>> create_plan(const Costmap& costmap, const Pose2& start,
                                                     const Pose2& goal) override;
  [[nodiscard]] std::string type() const override { return "nav2_smac_planner::SmacPlannerHybrid"; }
  [[nodiscard]] int expanded() const { return expanded_; }

 private:
  Parameter<std::string> motion_model_;
  Parameter<double> minimum_turning_radius_;
  Parameter<std::int64_t> angle_bins_;
  Parameter<double> cost_penalty_;
  Parameter<double> non_straight_penalty_;
  Parameter<double> reverse_penalty_;
  Parameter<double> tolerance_;
  Parameter<bool> allow_unknown_;
  Parameter<std::int64_t> max_iterations_;
  int expanded_ = 0;
};

}  // namespace krosc::nodes::nav2
