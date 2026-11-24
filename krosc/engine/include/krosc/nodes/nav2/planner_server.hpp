#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <optional>
#include <stdexcept>

#include "krosc/action.hpp"
#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/tf.hpp"
#include "krosc/tf_ros.hpp"

#include "krosc/nodes/nav2/costmap_2d_ros.hpp"
#include "krosc/nodes/nav2/global_planner.hpp"

namespace krosc::nodes::nav2 {

// nav2 planner_server 흉내: compute_path_to_pose 액션을 받아 NavPlannerPlugin 플러그인으로 계획하고
// plan(nav_msgs/Path)을 latch로 낸다. costmap은 별도 Costmap2DROS 노드에서 읽는다.
class PlannerServer final : public Node {
 public:
  explicit PlannerServer(Context& context, Costmap2DROS& global_costmap,
                         std::string name = "planner_server",
                         std::map<std::string, ParamValue> parameters = {},
                         PlannerRegistry registry = builtin_planners());
  ~PlannerServer() override;

  PlannerServer(const PlannerServer&) = delete;
  PlannerServer& operator=(const PlannerServer&) = delete;

  [[nodiscard]] msgs::nav2::ComputePathToPoseResult compute_path(
      const msgs::nav2::ComputePathToPoseGoal& goal);
  [[nodiscard]] std::vector<std::string> planner_ids() const;

 private:
  [[nodiscard]] std::optional<Pose2> pose_in_frame(const msgs::PoseStamped& pose, const std::string& frame) const;

  Costmap2DROS& costmap_;
  PlannerRegistry registry_;
  std::map<std::string, std::unique_ptr<NavPlannerPlugin>> planners_;
  Parameter<std::vector<std::string>> planner_plugins_;
  Parameter<std::string> plan_topic_;
  Buffer buffer_;
  std::unique_ptr<TransformListener> listener_;
  std::unique_ptr<ActionServer<msgs::nav2::ComputePathToPoseGoal, msgs::nav2::ComputePathToPoseFeedback,
                               msgs::nav2::ComputePathToPoseResult>>
      action_;
};

}  // namespace krosc::nodes::nav2
