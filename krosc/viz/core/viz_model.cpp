#include "krosc/viz/viz_model.hpp"

#include <chrono>
#include <utility>

#include "krosc/msgs/rcl_interfaces.hpp"
#include "krosc/parameter_rpc.hpp"
#include "krosc/control/controller_manager.hpp"

namespace krosc::viz {

namespace {
constexpr const char* kRosoutTopic = "rosout";
}

VizModel::VizModel(Context& context)
    : node_(context, "Viz"),
      listener_(buffer_, node_),
      nav_client_(std::make_unique<NavClient>(context, msgs::nav2::NAVIGATE_TO_POSE_ACTION)),
      through_client_(
          std::make_unique<ThroughClient>(context, msgs::nav2::NAVIGATE_THROUGH_POSES_ACTION)),
      path_client_(
          std::make_unique<PathClient>(context, msgs::nav2::COMPUTE_PATH_TO_POSE_ACTION)),
      path_through_client_(std::make_unique<PathThroughClient>(
          context, msgs::nav2::COMPUTE_PATH_THROUGH_POSES_ACTION)) {
  subscribe_latest<msgs::LaserScan>("scan", scan_, QOS_SENSOR_DATA);
  subscribe_latest<msgs::Imu>("imu", imu_, QOS_SENSOR_DATA);
  subscribe_latest<msgs::Odometry>("odom", odom_, QOS_SENSOR_DATA);
  subscribe_latest<msgs::Odometry>("odometry/filtered", filtered_, QOS_SENSOR_DATA);
  subscribe_latest<msgs::OccupancyGrid>("map", map_, QOS_LATCHED);
  subscribe_latest<msgs::OccupancyGrid>("global_costmap/costmap", global_costmap_, QOS_LATCHED);
  subscribe_latest<msgs::OccupancyGrid>("local_costmap/costmap", local_costmap_, QOS_LATCHED);
  subscribe_latest<msgs::Path>("plan", plan_, QOS_LATCHED);
  subscribe_latest<msgs::Path>("local_plan", local_plan_, QOS_SENSOR_DATA);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_["diagnostics"] =
        Stat{std::string(msgs::message_type_name<msgs::DiagnosticArray>()), 0, 0.0, 0.0};
  }
  node_.subscribe<msgs::DiagnosticArray>(
      "diagnostics", [this](const msgs::DiagnosticArray& message) { store_diagnostics(message); },
      QOS_DEFAULT);
  subscribe_latest<msgs::String>(ROBOT_DESCRIPTION_TOPIC_NAME, robot_description_, QOS_LATCHED);
  subscribe_latest<msgs::String>("map_list", map_list_, QOS_LATCHED);
  subscribe_latest<msgs::String>("map_status", map_status_, QOS_LATCHED);
  subscribe_latest<msgs::String>("goal_status", goal_status_, QOS_LATCHED);
  subscribe_latest<msgs::String>("collision_status", collision_status_, QOS_LATCHED);
  subscribe_latest<msgs::String>("arm_status", arm_status_, QOS_DEFAULT);
  subscribe_latest<msgs::String>("rl_nav_status", rl_nav_status_, QOS_DEFAULT);
  subscribe_latest<msgs::String>("moveit_status", moveit_status_, QOS_DEFAULT);
  subscribe_latest<msgs::String>("moveit_state", moveit_state_, QOS_DEFAULT);
  subscribe_latest<msgs::PoseStamped>("moveit_target_pose", moveit_target_pose_, QOS_DEFAULT);
  subscribe_latest<msgs::Path>("moveit_arm", moveit_arm_, QOS_DEFAULT);
  subscribe_latest<msgs::Path>("moveit_path", moveit_path_, QOS_DEFAULT);
  subscribe_latest<msgs::JointState>("joint_states", joint_states_, QOS_DEFAULT);
  subscribe_latest<msgs::String>("slam_status", slam_status_, QOS_LATCHED);
  subscribe_stats_only(TF_TOPIC_NAME, "tf2_msgs/TFMessage", QOS_TF);
  subscribe_stats_only(TF_STATIC_TOPIC_NAME, "tf2_msgs/TFMessage", QOS_TF_STATIC);
  subscribe_stats_only(kRosoutTopic, "rcl_interfaces/Log", QOS_DEFAULT);
  node_.subscribe<msgs::Log>(kRosoutTopic, [this](const msgs::Log& log) { store_log(log); }, QOS_DEFAULT);
}

VizModel::~VizModel() {
  nav_client_.reset();
  through_client_.reset();
  path_client_.reset();
  path_through_client_.reset();
  node_.shutdown();
}

void VizModel::subscribe_stats_only(const std::string& topic, const std::string& type, QoS qos) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_[topic] = Stat{type, 0, 0.0, 0.0};
  }
  if (topic == TF_TOPIC_NAME) {
    node_.subscribe<msgs::TFMessage>(
        topic,
        [this, topic](const msgs::TFMessage&) {
          const double now = monotonic_now();
          std::lock_guard<std::mutex> lock(mutex_);
          auto& stat = stats_[topic];
          ++stat.count;
          if (stat.count == 1) {
            stat.first = now;
          }
          stat.last = now;
        },
        qos);
  } else if (topic == TF_STATIC_TOPIC_NAME) {
    node_.subscribe<msgs::TFMessage>(
        topic,
        [this, topic](const msgs::TFMessage&) {
          const double now = monotonic_now();
          std::lock_guard<std::mutex> lock(mutex_);
          auto& stat = stats_[topic];
          ++stat.count;
          if (stat.count == 1) {
            stat.first = now;
          }
          stat.last = now;
        },
        qos);
  } else if (topic == kRosoutTopic) {
    node_.subscribe<msgs::Log>(
        topic,
        [this, topic](const msgs::Log&) {
          const double now = monotonic_now();
          std::lock_guard<std::mutex> lock(mutex_);
          auto& stat = stats_[topic];
          ++stat.count;
          if (stat.count == 1) {
            stat.first = now;
          }
          stat.last = now;
        },
        qos);
  }
}

void VizModel::store_log(const msgs::Log& log) {
  std::lock_guard<std::mutex> lock(mutex_);
  logs_.push_back(log);
  while (logs_.size() > 50) {
    logs_.pop_front();
  }
}

void VizModel::store_diagnostics(const msgs::DiagnosticArray& message) {
  const double now = monotonic_now();
  std::lock_guard<std::mutex> lock(mutex_);
  auto merged = diagnostics_ ? std::make_shared<msgs::DiagnosticArray>(*diagnostics_)
                             : std::make_shared<msgs::DiagnosticArray>();
  merged->header = message.header;
  for (const auto& status : message.status) {
    bool replaced = false;
    for (auto& existing : merged->status) {
      if (existing.name == status.name) {
        existing = status;
        replaced = true;
        break;
      }
    }
    if (!replaced) {
      merged->status.push_back(status);
    }
  }
  diagnostics_ = std::move(merged);
  auto& stat = stats_["diagnostics"];
  ++stat.count;
  if (stat.count == 1) {
    stat.first = now;
  }
  stat.last = now;
}

double VizModel::monotonic_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void VizModel::set_fixed_frame(std::string frame) {
  fixed_frame_ = std::move(frame);
}

std::shared_ptr<const msgs::LaserScan> VizModel::scan() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return scan_;
}
std::shared_ptr<const msgs::Imu> VizModel::imu() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_;
}
std::shared_ptr<const msgs::Odometry> VizModel::odom() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return odom_;
}
std::shared_ptr<const msgs::Odometry> VizModel::filtered() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return filtered_;
}
std::shared_ptr<const msgs::OccupancyGrid> VizModel::map() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_;
}
std::shared_ptr<const msgs::OccupancyGrid> VizModel::global_costmap() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return global_costmap_;
}
std::shared_ptr<const msgs::OccupancyGrid> VizModel::local_costmap() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return local_costmap_;
}
std::shared_ptr<const msgs::Path> VizModel::plan() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return plan_;
}
std::shared_ptr<const msgs::Path> VizModel::local_plan() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return local_plan_;
}
std::shared_ptr<const msgs::DiagnosticArray> VizModel::diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return diagnostics_;
}
std::shared_ptr<const msgs::String> VizModel::robot_description() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_description_;
}

std::shared_ptr<const msgs::String> VizModel::map_list() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_list_;
}

std::shared_ptr<const msgs::String> VizModel::map_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_status_;
}

std::shared_ptr<const msgs::String> VizModel::goal_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return goal_status_;
}

std::shared_ptr<const msgs::String> VizModel::collision_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return collision_status_;
}

std::shared_ptr<const msgs::String> VizModel::arm_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return arm_status_;
}

std::shared_ptr<const msgs::String> VizModel::rl_nav_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rl_nav_status_;
}

std::shared_ptr<const msgs::String> VizModel::moveit_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return moveit_status_;
}

std::shared_ptr<const msgs::String> VizModel::moveit_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return moveit_state_;
}

std::shared_ptr<const msgs::PoseStamped> VizModel::moveit_target_pose() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return moveit_target_pose_;
}

std::shared_ptr<const msgs::Path> VizModel::moveit_arm() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return moveit_arm_;
}

std::shared_ptr<const msgs::Path> VizModel::moveit_path() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return moveit_path_;
}

std::shared_ptr<const msgs::JointState> VizModel::joint_states() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_states_;
}

std::shared_ptr<const msgs::String> VizModel::slam_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return slam_status_;
}


std::vector<msgs::Log> VizModel::logs() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {logs_.begin(), logs_.end()};
}

std::vector<VizModel::TopicRow> VizModel::topic_rows() const {
  const double now = monotonic_now();
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TopicRow> rows;
  rows.reserve(stats_.size());
  for (const auto& [topic, stat] : stats_) {
    const double span = stat.last - stat.first;
    const double hz = stat.count > 1 && span > 0.0 ? static_cast<double>(stat.count - 1) / span : 0.0;
    rows.push_back(TopicRow{topic, stat.type, stat.count, hz, stat.count > 0 && now - stat.last < 2.0});
  }
  return rows;
}

std::optional<msgs::PoseStamped> VizModel::pose_in_fixed(const std::string& frame, std::optional<double> stamp) const {
  if (frame.empty() || frame == fixed_frame_) {
    msgs::PoseStamped pose;
    pose.header.frame_id = fixed_frame_;
    pose.pose.orientation.w = 1.0;
    return pose;
  }
  try {
    const msgs::TransformStamped transform = buffer_.lookup_transform(fixed_frame_, frame, stamp);
    msgs::PoseStamped pose;
    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = fixed_frame_;
    pose.pose.position = {transform.transform.translation.x, transform.transform.translation.y,
                          transform.transform.translation.z};
    pose.pose.orientation = transform.transform.rotation;
    return pose;
  } catch (const TransformException&) {
    return std::nullopt;
  }
}

std::optional<msgs::TransformStamped> VizModel::transform_in_fixed(const std::string& frame) const {
  if (frame.empty()) {
    return std::nullopt;
  }
  try {
    return buffer_.lookup_transform(fixed_frame_, frame, std::nullopt);
  } catch (const TransformException&) {
    return std::nullopt;
  }
}

bool VizModel::set_parameters(const std::string& node_name,
                              const std::map<std::string, ParamValue>& values,
                              std::string& error) {
  using namespace msgs::rcl_interfaces;
  SetParametersRequest request;
  for (const auto& [name, value] : values) {
    msgs::rcl_interfaces::Parameter parameter;
    parameter.name = name;
    parameter.value = parameter_value_message(value);
    request.parameters.push_back(std::move(parameter));
  }
  try {
    const auto response =
        node_.call_service<SetParametersRequest, SetParametersResponse>(
            node_name + "/set_parameters", request);
    for (const auto& result : response.results) {
      if (!result.successful) {
        error = result.reason;
        return false;
      }
    }
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

std::optional<ParamValue> VizModel::get_parameter(const std::string& node_name,
                                                  const std::string& name) {
  using namespace msgs::rcl_interfaces;
  GetParametersRequest request;
  request.names = {name};
  try {
    const auto response =
        node_.call_service<GetParametersRequest, GetParametersResponse>(
            node_name + "/get_parameters", request);
    if (response.values.empty() ||
        response.values[0].type == static_cast<std::uint8_t>(ParameterType::NotSet)) {
      return std::nullopt;
    }
    return param_value_from_message(response.values[0]);
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

bool VizModel::switch_controller(const std::vector<std::string>& start,
                                 const std::vector<std::string>& stop, int strictness,
                                 std::string& error) {
  krosc::control::SwitchControllerRequest request;
  request.start_controllers = start;
  request.stop_controllers = stop;
  request.strictness = strictness;
  try {
    const auto response =
        node_.call_service<krosc::control::SwitchControllerRequest,
                           krosc::control::SwitchControllerResponse>(
            "controller_manager/switch_controller", request);
    error = response.message;
    return response.ok;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

bool VizModel::trigger(const std::string& service, std::string& message) {
  try {
    const auto response = node_.call_service<msgs::TriggerRequest, msgs::TriggerResponse>(
        service, msgs::TriggerRequest{});
    message = response.message;
    return response.success;
  } catch (const std::exception& exception) {
    message = exception.what();
    return false;
  }
}

void VizModel::publish_cmd_vel(double linear, double angular) {
  msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  // twist_mux의 teleop priority(100)를 타도록 cmd_vel_teleop로 보낸다.
  // cmd_vel로 직접 보내면 smoother/collision monitor 출력과 경쟁한다.
  node_.publish("cmd_vel_teleop", twist, QOS_COMMAND);
}

bool VizModel::navigate_to_pose(const msgs::PoseStamped& goal, std::string& error) {
  try {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    if (through_handle_) {
      (void)through_handle_->cancel();
      through_handle_.reset();
    }
    nav_handle_ = nav_client_->send_goal(msgs::nav2::NavigateToPoseGoal{goal, ""});
    if (!nav_handle_->accepted()) {
      error = "navigate_to_pose goal이 거부됐다";
      nav_handle_.reset();
      return false;
    }
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

bool VizModel::compute_path_to_pose(const msgs::PoseStamped& goal, std::string& error) {
  try {
    const auto handle = path_client_->send_goal(msgs::nav2::ComputePathToPoseGoal{goal, {}, "", false});
    if (!handle->accepted()) {
      error = "compute_path_to_pose goal이 거부됐다";
      return false;
    }
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

bool VizModel::navigate_through_poses(const std::vector<msgs::PoseStamped>& goals,
                                      std::string& error) {
  if (goals.empty()) {
    error = "웨이포인트가 없다";
    return false;
  }
  try {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    if (nav_handle_) {
      (void)nav_handle_->cancel();
      nav_handle_.reset();
    }
    through_handle_ =
        through_client_->send_goal(msgs::nav2::NavigateThroughPosesGoal{goals, ""});
    if (!through_handle_->accepted()) {
      error = "navigate_through_poses goal이 거부됐다";
      through_handle_.reset();
      return false;
    }
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

bool VizModel::compute_path_through_poses(const std::vector<msgs::PoseStamped>& goals,
                                          std::string& error) {
  if (goals.empty()) {
    error = "웨이포인트가 없다";
    return false;
  }
  try {
    const auto handle = path_through_client_->send_goal(
        msgs::nav2::ComputePathThroughPosesGoal{goals, {}, "", false});
    if (!handle->accepted()) {
      error = "compute_path_through_poses goal이 거부됐다";
      return false;
    }
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

bool VizModel::cancel_navigation(std::string& error) {
  try {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    if (nav_handle_) {
      (void)nav_handle_->cancel();
      nav_handle_.reset();
    }
    if (through_handle_) {
      (void)through_handle_->cancel();
      through_handle_.reset();
    }
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace krosc::viz
