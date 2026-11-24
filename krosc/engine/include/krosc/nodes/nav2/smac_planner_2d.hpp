#pragma once

#include "krosc/nodes/nav2/global_planner.hpp"

namespace krosc::nodes::nav2 {

class SmacPlanner2D final : public NavPlannerPlugin {
 public:
  void configure(Node& node, const std::string& name) override;
  std::vector<std::pair<double, double>> create_plan(const Costmap& costmap, const Pose2& start,
                                                     const Pose2& goal) override;
  [[nodiscard]] std::string type() const override { return "nav2_smac_planner::SmacPlanner2D"; }
  [[nodiscard]] int expanded() const { return expanded_; }

 private:
  std::vector<std::pair<double, double>> smooth(const Costmap& costmap,
                                                const std::vector<std::pair<double, double>>& points) const;
  Parameter<double> tolerance_;
  Parameter<bool> allow_unknown_;
  Parameter<double> cost_travel_multiplier_;
  Parameter<std::int64_t> max_iterations_;
  Parameter<bool> smooth_path_;
  Parameter<double> w_smooth_;
  Parameter<double> w_data_;
  Parameter<std::int64_t> smoother_iterations_;
  int expanded_ = 0;
};

}  // namespace krosc::nodes::nav2
