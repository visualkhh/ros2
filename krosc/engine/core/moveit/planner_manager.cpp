#include "krosc/moveit/planner_manager.hpp"

namespace krosc::moveit {

PlanningResult MoveItPlannerPlugin::run(const PlanningRequest& request, bool cartesian) const {
  PlanningResult result;
  if (chain_ == nullptr) {
    result.error = "URDF 체인이 없다";
    return result;
  }
  manipulation::PlanOptions options = options_;
  options.max_joint_speed = request.max_joint_speed;
  options.max_joint_accel = request.max_joint_accel;
  options.sample_time = request.sample_time;
  options.cartesian_speed = request.cartesian_speed;
  options.cartesian_accel = request.cartesian_accel;
  options.cartesian = cartesian;
  auto trajectory = manipulation::plan_trajectory(*chain_, request.start, request.goal, options,
                                                  result.error);
  if (!trajectory) {
    return result;
  }
  result.success = true;
  result.trajectory = std::move(*trajectory);
  return result;
}

}  // namespace krosc::moveit
