#include "krosc/nodes/nav2/planner_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>

#include "krosc/tf/math.hpp"
#include "krosc/qos.hpp"

namespace krosc::nodes::nav2 {

namespace {
double yaw_of(const msgs::Quaternion& q) { return tfmath::yaw_of(q); }
}  // namespace

PlannerServer::PlannerServer(Context& context, Costmap2DROS& global_costmap, std::string name,
                             std::map<std::string, ParamValue> parameters, PlannerRegistry registry)
    : Node(context, std::move(name), std::move(parameters)),
      costmap_(global_costmap),
      registry_(std::move(registry)),
      planner_plugins_(declare_parameter<std::vector<std::string>>(
          "planner_plugins", {"GridBased"},
          [](const std::vector<std::string>& values) { return !values.empty(); })),
      plan_topic_(declare_parameter<std::string>("plan_topic", "plan")),
      buffer_() {
  listener_ = std::make_unique<TransformListener>(buffer_, *this);
  for (const auto& id : planner_plugins_.value()) {
    const auto kind = declare_parameter<std::string>(
        id + ".plugin", "nav2_navfn_planner::NavfnPlanner",
        [this](const std::string& value) { return registry_.contains(value); }).value();
    auto planner = registry_.create(kind);
    if (!planner) {
      throw std::runtime_error("planner plugin not found: " + kind);
    }
    planner->configure(*this, id);
    planners_[id] = std::move(planner);
  }
  action_ = std::make_unique<ActionServer<msgs::nav2::ComputePathToPoseGoal,
                                          msgs::nav2::ComputePathToPoseFeedback,
                                          msgs::nav2::ComputePathToPoseResult>>(
      context, msgs::nav2::COMPUTE_PATH_TO_POSE_ACTION,
      [this](ServerGoalHandle<msgs::nav2::ComputePathToPoseGoal, msgs::nav2::ComputePathToPoseFeedback>& handle) {
        return compute_path(handle.goal());
      });
  logger().info("planner_server: %zu plugins, costmap=%s", planners_.size(), costmap_.name().c_str());
}

PlannerServer::~PlannerServer() {
  if (action_) {
    action_->shutdown();
  }
}

std::vector<std::string> PlannerServer::planner_ids() const {
  std::vector<std::string> ids;
  for (const auto& [id, planner] : planners_) {
    (void)planner;
    ids.push_back(id);
  }
  return ids;
}

std::optional<Pose2> PlannerServer::pose_in_frame(const msgs::PoseStamped& pose, const std::string& frame) const {
  const double yaw = yaw_of(pose.pose.orientation);
  if (pose.header.frame_id.empty() || pose.header.frame_id == frame) {
    return Pose2{pose.pose.position.x, pose.pose.position.y, yaw};
  }
  try {
    const msgs::TransformStamped transform = buffer_.lookup_transform(frame, pose.header.frame_id);
    const double offset = yaw_of(transform.transform.rotation);
    const double c = std::cos(offset);
    const double s = std::sin(offset);
    return Pose2{transform.transform.translation.x + c * pose.pose.position.x - s * pose.pose.position.y,
                 transform.transform.translation.y + s * pose.pose.position.x + c * pose.pose.position.y,
                 offset + yaw};
  } catch (const TransformException&) {
    return std::nullopt;
  }
}

msgs::nav2::ComputePathToPoseResult PlannerServer::compute_path(
    const msgs::nav2::ComputePathToPoseGoal& goal) {
  const auto started = std::chrono::steady_clock::now();
  msgs::nav2::ComputePathToPoseResult result;
  const std::string planner_id =
      goal.planner_id.empty() ? (planners_.empty() ? std::string{} : planners_.begin()->first) : goal.planner_id;
  auto planner_it = planners_.find(planner_id);
  if (planner_it == planners_.end()) {
    result.error_code = msgs::nav2::ComputePathToPoseResult::INVALID_PLANNER;
    result.error_msg = "planner '" + planner_id + "'가 없다";
    return result;
  }
  const auto costmap = costmap_.costmap();
  if (!costmap) {
    result.error_code = msgs::nav2::ComputePathToPoseResult::UNKNOWN;
    result.error_msg = "global_costmap에 지도가 아직 없다";
    return result;
  }
  const std::string frame = costmap_.global_frame();
  Pose2 start;
  if (goal.use_start) {
    const auto parsed = pose_in_frame(goal.start, frame);
    if (!parsed) {
      result.error_code = msgs::nav2::ComputePathToPoseResult::TF_ERROR;
      result.error_msg = "시작 좌표계 변환 실패";
      return result;
    }
    start = *parsed;
  } else {
    const auto pose = costmap_.get_robot_pose();
    if (!pose) {
      result.error_code = msgs::nav2::ComputePathToPoseResult::TF_ERROR;
      result.error_msg = "로봇 위치 TF가 없다";
      return result;
    }
    start = Pose2{pose->x, pose->y, pose->theta};
  }
  const auto target = pose_in_frame(goal.goal, frame);
  if (!target) {
    result.error_code = msgs::nav2::ComputePathToPoseResult::TF_ERROR;
    result.error_msg = "목표 좌표계 변환 실패";
    return result;
  }
  try {
    std::vector<std::pair<double, double>> points = planner_it->second->create_plan(*costmap, start, *target);
    if (points.empty()) {
      points.emplace_back(target->x, target->y);
    }
    points.back() = {target->x, target->y};
    const double stamp = now();
    msgs::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = frame;
    path.poses.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      const double yaw = index + 1 < points.size()
                             ? std::atan2(points[index + 1].second - points[index].second,
                                          points[index + 1].first - points[index].first)
                             : target->yaw;
      msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = {points[index].first, points[index].second, 0.0};
      pose.pose.orientation = tfmath::from_yaw(yaw);
      path.poses.push_back(std::move(pose));
    }
    if (path.poses.size() > 2) {
      path.poses[path.poses.size() - 2].pose.orientation = tfmath::from_yaw(target->yaw);
    }
    publish(plan_topic_.value(), path, QOS_LATCHED);
    result.path = path;
  } catch (const PlannerError& error) {
    result.error_code = error.code;
    result.error_msg = error.what();
    logger().warn("경로 계획 실패 (%d): %s", error.code, error.what());
  } catch (const std::exception& error) {
    result.error_code = msgs::nav2::ComputePathToPoseResult::UNKNOWN;
    result.error_msg = error.what();
    logger().warn("경로 계획 실패: %s", error.what());
  }
  result.planning_time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  return result;
}

}  // namespace krosc::nodes::nav2
