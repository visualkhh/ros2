#pragma once

#include <queue>
#include <vector>

#include "krosc/nodes/nav2/global_planner.hpp"

namespace krosc::nodes::nav2 {

class NavfnPlanner final : public GlobalPlanner {
 public:
  void configure(Node& node, const std::string& name) override;
  std::vector<std::pair<double, double>> create_plan(const Costmap& costmap, const Pose2& start,
                                                     const Pose2& goal) override;
  [[nodiscard]] int expanded() const { return expanded_; }

 private:
  Parameter<double> tolerance_;
  Parameter<bool> use_astar_;
  Parameter<bool> allow_unknown_;
  int expanded_ = 0;
};

class ThetaStarPlanner final : public GlobalPlanner {
 public:
  void configure(Node& node, const std::string& name) override;
  std::vector<std::pair<double, double>> create_plan(const Costmap& costmap, const Pose2& start,
                                                     const Pose2& goal) override;
  [[nodiscard]] int expanded() const { return expanded_; }

 private:
  Parameter<double> w_euc_cost_;
  Parameter<double> w_traversal_cost_;
  Parameter<bool> allow_unknown_;
  Parameter<double> tolerance_;
  int expanded_ = 0;
};

class SmacPlanner2D final : public GlobalPlanner {
 public:
  void configure(Node& node, const std::string& name) override;
  std::vector<std::pair<double, double>> create_plan(const Costmap& costmap, const Pose2& start,
                                                     const Pose2& goal) override;
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

class SmacPlannerHybrid final : public GlobalPlanner {
 public:
  void configure(Node& node, const std::string& name) override;
  std::vector<std::pair<double, double>> create_plan(const Costmap& costmap, const Pose2& start,
                                                     const Pose2& goal) override;
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
