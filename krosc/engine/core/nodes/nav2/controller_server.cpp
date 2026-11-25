#include "krosc/nodes/nav2/controller_server.hpp"

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

bool ControllerServer::ProgressChecker::check(const Pose2& pose, double now) {
  if (!baseline) {
    baseline = std::make_pair(pose, now);
    return true;
  }
  const bool moved =
      std::hypot(pose.x - baseline->first.x, pose.y - baseline->first.y) > required_movement_radius;
  const bool turned = std::abs(wrap_angle(pose.yaw - baseline->first.yaw)) > required_angle;
  if (moved || turned) {
    baseline = std::make_pair(pose, now);
    return true;
  }
  return now - baseline->second <= movement_time_allowance;
}

ControllerServer::ControllerServer(Context& context, Costmap2DROS& local_costmap, std::string name,
                                   std::map<std::string, ParamValue> parameters,
                                   ControllerRegistry registry)
    : Node(context, std::move(name), std::move(parameters)),
      costmap_(local_costmap),
      registry_(std::move(registry)),
      controller_plugins_(declare_parameter<std::vector<std::string>>(
          "controller_plugins", {"FollowPath"},
          [](const std::vector<std::string>& values) { return !values.empty(); })),
      controller_frequency_(declare_parameter<double>("controller_frequency", 20.0,
                                                      [](double value) { return value > 0.0; })),
      odom_topic_(declare_parameter<std::string>("odom_topic", "odom")),
      cmd_vel_topic_(declare_parameter<std::string>("cmd_vel_topic", "cmd_vel")),
      local_plan_topic_(declare_parameter<std::string>("local_plan_topic", "local_plan")),
      final_rotation_gain_(declare_parameter<double>("final_rotation_gain", 1.5,
                                                     [](double value) { return value > 0.0; })),
      final_rotation_speed_(declare_parameter<double>("final_rotation_speed", 0.8,
                                                      [](double value) { return value > 0.0; })),
      max_linear_accel_(declare_parameter<double>("max_linear_accel", 0.0,
                                                 [](double value) { return value >= 0.0; })),
      max_angular_accel_(declare_parameter<double>("max_angular_accel", 0.0,
                                                  [](double value) { return value >= 0.0; })),
      xy_goal_tolerance_(declare_parameter<double>("general_goal_checker.xy_goal_tolerance", 0.25,
                                                   [](double value) { return value > 0.0; })),
      yaw_goal_tolerance_(declare_parameter<double>("general_goal_checker.yaw_goal_tolerance", 0.25,
                                                    [](double value) { return value > 0.0; })),
      startup_wait_(declare_parameter<double>("startup_wait", 1.0,
                                              [](double value) { return value >= 0.0; })) {
  progress_checker_.required_movement_radius =
      declare_parameter<double>("progress_checker.required_movement_radius", 0.5,
                                [](double value) { return value > 0.0; })
          .value();
  progress_checker_.movement_time_allowance =
      declare_parameter<double>("progress_checker.movement_time_allowance", 10.0,
                                [](double value) { return value > 0.0; })
          .value();
  progress_checker_.required_angle =
      declare_parameter<double>("progress_checker.required_angle", 0.5,
                                [](double value) { return value > 0.0; })
          .value();
  for (const auto& id : controller_plugins_.value()) {
    const auto kind = declare_parameter<std::string>(
        id + ".plugin", "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        [this](const std::string& value) { return registry_.contains(value); }).value();
    auto controller = registry_.create(kind);
    if (!controller) {
      throw std::runtime_error("controller plugin not found: " + kind);
    }
    controller->set_registry(&registry_);
    // nav2의 configure(parent, ...)와 같이 서버가 아니라 parent Node로 넘긴다.
    // *this는 ControllerServer지만 플러그인 계약은 Node&다 (슬라이싱 없음).
    controller->configure(*this, id);
    controllers_[id] = std::move(controller);
    logger().info("controller %s: %s", id.c_str(), kind.c_str());
  }
  odom_subscription_ = subscribe<msgs::Odometry>(
      odom_topic_.value(), [this](const msgs::Odometry& odom) { on_odom(odom); }, QOS_SENSOR_DATA);
  tick_timer_ = create_timer(1.0 / controller_frequency_.value(), [this] {
    {
      std::lock_guard<std::mutex> lock(tick_mutex_);
      ++tick_seq_;
    }
    tick_cv_.notify_all();
  });
  action_ = std::make_unique<ActionServer<msgs::nav2::FollowPathGoal, msgs::nav2::FollowPathFeedback,
                                          msgs::nav2::FollowPathResult>>(
      context, msgs::nav2::FOLLOW_PATH_ACTION,
      [this](ServerGoalHandle<msgs::nav2::FollowPathGoal, msgs::nav2::FollowPathFeedback>& handle) {
        return run(handle.goal(), &handle);
      },
      [this](const msgs::nav2::FollowPathGoal& goal) {
        (void)goal;
        preempt_.store(true);
        return true;
      });
}

ControllerServer::~ControllerServer() {
  if (action_) {
    action_->shutdown();
  }
}

void ControllerServer::on_odom(const msgs::Odometry& odom) {
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  velocity_ = Velocity{odom.twist.twist.linear.x, odom.twist.twist.angular.z};
}

void ControllerServer::stop() {
  last_command_ = Velocity{};
  publish(cmd_vel_topic_.value(), msgs::Twist{}, QOS_COMMAND);
}

msgs::nav2::FollowPathResult ControllerServer::fail(int code, const std::string& message) {
  stop();
  logger().warn("경로 따라가기 실패 (%d): %s", code, message.c_str());
  msgs::nav2::FollowPathResult result;
  result.error_code = code;
  result.error_msg = message;
  return result;
}

std::vector<Pose2> ControllerServer::transform_path(const std::vector<Pose2>& path,
                                                    const std::string& source,
                                                    const std::string& target) const {
  if (source == target) {
    return path;
  }
  const msgs::TransformStamped transform = costmap_.buffer().lookup_transform(target, source);
  const double yaw = yaw_of(transform.transform.rotation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::vector<Pose2> result;
  result.reserve(path.size());
  for (const auto& point : path) {
    result.push_back(Pose2{transform.transform.translation.x + c * point.x - s * point.y,
                           transform.transform.translation.y + s * point.x + c * point.y,
                           point.yaw + yaw});
  }
  return result;
}

msgs::nav2::FollowPathResult ControllerServer::follow_path(
    const msgs::nav2::FollowPathGoal& goal) {
  return run(goal, nullptr);
}

msgs::nav2::FollowPathResult ControllerServer::run(
    const msgs::nav2::FollowPathGoal& goal,
    ServerGoalHandle<msgs::nav2::FollowPathGoal, msgs::nav2::FollowPathFeedback>* handle) {
  msgs::nav2::FollowPathResult result;
  const std::string controller_id =
      goal.controller_id.empty()
          ? (controller_plugins_.value().empty() ? std::string{} : controller_plugins_.value().front())
          : goal.controller_id;
  auto controller_it = controllers_.find(controller_id);
  if (controller_it == controllers_.end()) {
    return fail(msgs::nav2::FollowPathResult::INVALID_CONTROLLER,
                "컨트롤러 '" + controller_id + "'가 없다");
  }
  NavControllerPlugin& controller = *controller_it->second;
  const std::string frame = costmap_.global_frame();
  if (goal.path.poses.empty()) {
    return fail(msgs::nav2::FollowPathResult::INVALID_PATH, "빈 경로");
  }
  const std::string path_frame = goal.path.header.frame_id.empty() ? frame : goal.path.header.frame_id;
  if (!costmap_.buffer().can_transform(frame, path_frame)) {
    return fail(msgs::nav2::FollowPathResult::TF_ERROR,
                "경로 좌표계 " + path_frame + "를 costmap 좌표계 " + frame + "로 바꿀 수 없다");
  }
  std::vector<Pose2> original;
  original.reserve(goal.path.poses.size());
  for (const auto& pose : goal.path.poses) {
    original.push_back(
        Pose2{pose.pose.position.x, pose.pose.position.y, yaw_of(pose.pose.orientation)});
  }
  const double started = now();
  preempt_.store(false);
  progress_checker_.reset();
  const double period = 1.0 / controller_frequency_.value();
  std::optional<std::tuple<double, double, double>> cached_key;
  std::vector<Pose2> cached_path = original;
  std::int64_t tick = 0;
  for (;;) {
    std::uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(tick_mutex_);
      sequence = tick_seq_;
    }
    {
      std::unique_lock<std::mutex> lock(tick_mutex_);
      tick_cv_.wait_for(lock, std::chrono::milliseconds(200), [&] {
        return tick_seq_ != sequence || preempt_.load();
      });
    }
    if (handle && handle->is_cancel_requested()) {
      stop();
      result.error_code = msgs::nav2::FollowPathResult::NONE;
      result.error_msg = "취소";
      return result;
    }
    if (preempt_.load()) {
      result.error_code = msgs::nav2::FollowPathResult::NONE;
      result.error_msg = "새 경로로 바뀌었다";
      return result;
    }
    const auto costmap = costmap_.costmap();
    const auto pose_value = costmap_.get_robot_pose();
    if (!costmap || !pose_value) {
      if (now() - started <= startup_wait_.value()) {
        continue;
      }
      return fail(msgs::nav2::FollowPathResult::TF_ERROR,
                  "local_costmap이 없거나 로봇 위치(TF)를 모른다");
    }
    const Pose2 pose{pose_value->x, pose_value->y, pose_value->theta};
    std::vector<Pose2> path;
    if (path_frame == frame) {
      path = original;
    } else {
      try {
        const auto transform = costmap_.buffer().lookup_transform(frame, path_frame).transform;
        const std::tuple<double, double, double> key{
            transform.translation.x, transform.translation.y, yaw_of(transform.rotation)};
        if (!cached_key || key != *cached_key) {
          cached_key = key;
          cached_path = transform_path(original, path_frame, frame);
        }
        path = cached_path;
      } catch (const TransformException& error) {
        return fail(msgs::nav2::FollowPathResult::TF_ERROR, error.what());
      }
    }
    const Pose2& last_pose = path.back();
    const double dist_to_goal = std::hypot(last_pose.x - pose.x, last_pose.y - pose.y);
    const double yaw_error = wrap_angle(last_pose.yaw - pose.yaw);
    if (dist_to_goal <= xy_goal_tolerance_.value() &&
        std::abs(yaw_error) <= yaw_goal_tolerance_.value()) {
      stop();
      result.error_code = msgs::nav2::FollowPathResult::NONE;
      return result;
    }
    if (dist_to_goal <= xy_goal_tolerance_.value() &&
        std::abs(yaw_error) > yaw_goal_tolerance_.value()) {
      if (!progress_checker_.check(pose, now())) {
        return fail(msgs::nav2::FollowPathResult::FAILED_TO_MAKE_PROGRESS,
                    std::to_string(progress_checker_.movement_time_allowance) +
                        "초 동안 움직이지 못했다");
      }
      const double w = std::max(-final_rotation_speed_.value(),
                                std::min(final_rotation_speed_.value(),
                                         final_rotation_gain_.value() * yaw_error));
      publish_command(0.0, w, controller.local_plan(), dist_to_goal, frame, handle, tick);
      ++tick;
      continue;
    }
    if (!progress_checker_.check(pose, now())) {
      return fail(msgs::nav2::FollowPathResult::FAILED_TO_MAKE_PROGRESS,
                  std::to_string(progress_checker_.movement_time_allowance) +
                      "초 동안 움직이지 못했다");
    }
    Velocity velocity;
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      velocity = velocity_;
    }
    Velocity command;
    try {
      command = controller.compute_velocity_commands(pose, velocity, path, *costmap, period);
    } catch (const ControllerError& error) {
      return fail(error.code, error.what());
    }
    if (dist_to_goal < 0.6 && command.linear > 0.0) {
      const double scale = std::max(0.2, dist_to_goal / 0.6);
      command.linear *= scale;
      command.angular *= scale;
    }
    publish_command(command.linear, command.angular, controller.local_plan(), dist_to_goal, frame,
                    handle, tick);
    ++tick;
  }
}

void ControllerServer::publish_command(
    double linear, double angular, const std::vector<Pose2>& local_plan, double distance,
    const std::string& frame,
    ServerGoalHandle<msgs::nav2::FollowPathGoal, msgs::nav2::FollowPathFeedback>* handle,
    std::int64_t tick) {
  const double period = 1.0 / controller_frequency_.value();
  const double max_dv = max_linear_accel_.value() * period;
  const double max_dw = max_angular_accel_.value() * period;
  if (max_dv > 0.0) {
    linear = std::max(last_command_.linear - max_dv, std::min(last_command_.linear + max_dv, linear));
  }
  if (max_dw > 0.0) {
    angular =
        std::max(last_command_.angular - max_dw, std::min(last_command_.angular + max_dw, angular));
  }
  last_command_ = Velocity{linear, angular};
  msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  publish(cmd_vel_topic_.value(), std::move(twist), QOS_COMMAND);
  const double stamp = now();
  if (tick % 2 == 0) {
    msgs::Path path;
    path.header = msgs::Header{stamp, frame};
    path.poses.reserve(local_plan.size());
    for (const auto& point : local_plan) {
      msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = {point.x, point.y, 0.0};
      pose.pose.orientation = tfmath::from_yaw(point.yaw);
      path.poses.push_back(std::move(pose));
    }
    publish(local_plan_topic_.value(), std::move(path), QOS_DEFAULT);
  }
  if (handle && tick % 4 == 0) {
    msgs::nav2::FollowPathFeedback feedback;
    feedback.distance_to_goal = distance;
    feedback.speed = linear;
    handle->publish_feedback(feedback);
  }
}

}  // namespace krosc::nodes::nav2
