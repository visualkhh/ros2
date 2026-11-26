#include "engine.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "krosc/control/control_manager.hpp"
#include "krosc/control/controller_manager.hpp"
#include "krosc/nodes/cartographer.hpp"
#include "krosc/nodes/collision_monitor.hpp"
#include "krosc/nodes/ekf.hpp"
#include "krosc/nodes/map_catalog.hpp"
#include "krosc/nodes/map_saver.hpp"
#include "krosc/nodes/map_server.hpp"
#include "krosc/nodes/moveit/move_group.hpp"
#include "krosc/nodes/nav2/behavior_server.hpp"
#include "krosc/nodes/nav2/bt_navigator.hpp"
#include "krosc/nodes/nav2/controller_server.hpp"
#include "krosc/nodes/nav2/costmap/costmap_2d_ros.hpp"
#include "krosc/nodes/nav2/planner_server.hpp"
#include "krosc/nodes/robot_state_publisher.hpp"
#include "krosc/nodes/twist_mux.hpp"
#include "krosc/nodes/velocity_smoother.hpp"
#include "krosc/sensor/sensor_manager.hpp"

#include "controls/opencr/opencr_control.hpp"
#include "hardwares/lds03/lds_device.hpp"
#include "hardwares/opencr/opencr_device.hpp"
#include "nodes/robot_status.hpp"
#include "sensors/lds03/lds03_sensor.hpp"
#include "sensors/opencr/opencr_imu.hpp"
#include "sensors/realsense/realsense_sensor.hpp"

#include "hardwares/realsense/realsense_camera.hpp"

#ifdef KROSC_WS_ENABLE_RL_NAV
#include "nodes/rl_nav_controller.hpp"
#endif

namespace krosc_ws {

namespace {

std::string read_text(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) {
    return {};
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::vector<bool> ekf_config(std::initializer_list<int> enabled) {
  std::vector<bool> config(15, false);
  for (const int index : enabled) {
    if (index >= 0 && index < 15) {
      config[static_cast<std::size_t>(index)] = true;
    }
  }
  return config;
}

}  // namespace

Engine::Engine(krosc::Context& context, OpenCrDevice& opencr, LdsDevice* lidar,
               EngineOptions options, RealSenseCamera* camera)
    : context_(context), opencr_(opencr), lidar_(lidar), camera_(camera),
      options_(std::move(options)), urdf_(read_text(options_.urdf_path)) {
  if (urdf_.empty()) {
    throw std::runtime_error("Engine: URDF를 읽지 못했다: " + options_.urdf_path);
  }

  opencr_.start();
  if (lidar_ != nullptr) {
    lidar_->start();
  }
  if (camera_ != nullptr && options_.enable_camera) {
    camera_->start();
  }

  control_ = std::make_unique<OpenCrControl>(opencr_, options_.wheel_radius, options_.wheel_separation);
  imu_ = std::make_unique<OpenCrImu>(context_, opencr_, options_.imu_topic, options_.imu_frame);
  if (lidar_ != nullptr) {
    lds_sensor_ = std::make_unique<Lds03Sensor>(context_, *lidar_, options_.scan_topic,
                                                options_.scan_frame);
  }
  if (camera_ != nullptr && options_.enable_camera) {
    camera_sensor_ = std::make_unique<RealSenseSensor>(
        context_, *camera_, options_.camera_color_topic, options_.camera_depth_topic,
        options_.camera_color_frame, options_.camera_depth_frame, "diagnostics",
        options_.camera_depth_period);
  }
  if (options_.enable_status) {
    status_ = std::make_unique<RobotStatus>(context_, opencr_, lidar_);
  }

  std::vector<std::string> controller_plugins{"joint_state_broadcaster", "diff_drive_controller"};
  if (options_.enable_arm) {
    controller_plugins.push_back("joint_trajectory_controller");
    controller_plugins.push_back("arm_command_controller");
  }
  std::map<std::string, krosc::ParamValue> controller_params{
      {"update_rate", 100.0},
      {"controller_plugins", controller_plugins},
      {"joint_state_broadcaster.type",
       std::string("joint_state_broadcaster/JointStateBroadcaster")},
      {"joint_trajectory_controller.type",
       std::string("joint_trajectory_controller/JointTrajectoryController")},
      {"joint_trajectory_controller.joints",
       std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "gripper_left_joint"}},
      {"joint_trajectory_controller.goal_time_tolerance", 0.5},
      {"diff_drive_controller.type", std::string("diff_drive_controller/DiffDriveController")},
      {"diff_drive_controller.wheel_separation", options_.wheel_separation},
      {"diff_drive_controller.wheel_radius", options_.wheel_radius},
      {"diff_drive_controller.left_wheel", std::string("wheel_left_joint")},
      {"diff_drive_controller.right_wheel", std::string("wheel_right_joint")},
      {"diff_drive_controller.cmd_vel_topic", std::string("cmd_vel")},
      {"diff_drive_controller.odom_topic", std::string("odom")},
      {"diff_drive_controller.odom_frame", std::string("odom")},
      {"diff_drive_controller.base_frame", std::string("base_footprint")},
      {"diff_drive_controller.publish_tf", false},
      {"diff_drive_controller.command_timeout", 0.5},
      {"diff_drive_controller.odom_publish_rate", 50.0},
      {"arm_command_controller.type",
       std::string("forward_command_controller/ForwardCommandController")},
      {"arm_command_controller.joints",
       std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "gripper_left_joint"}},
      {"arm_command_controller.command_topic", std::string("arm_joint_command")},
      {"arm_command_controller.auto_activate", false}};
  if (!options_.enable_arm) {
    controller_params.erase("joint_trajectory_controller.type");
    controller_params.erase("joint_trajectory_controller.joints");
    controller_params.erase("joint_trajectory_controller.goal_time_tolerance");
    controller_params.erase("arm_command_controller.type");
    controller_params.erase("arm_command_controller.joints");
    controller_params.erase("arm_command_controller.command_topic");
    controller_params.erase("arm_command_controller.auto_activate");
  }
  controller_manager_ = std::make_unique<krosc::control::ControllerManager>(
      context_, "controller_manager", controller_params, *control_);

  ekf_ = std::make_unique<krosc::nodes::Ekf>(
      context_, "ekf_filter_node",
      std::map<std::string, krosc::ParamValue>{
          {"odom0", std::string("odom")},
          {"odom0_config", ekf_config({5, 6, 11})},
          {"frequency", 50.0},
          {"publish_tf", true}});

  if (options_.enable_robot_state_publisher) {
    robot_state_ = std::make_unique<krosc::nodes::RobotStatePublisher>(
        context_, "robot_state_publisher",
        std::map<std::string, krosc::ParamValue>{{"robot_description", urdf_}});
  }

  if (options_.enable_moveit) {
    move_group_ = std::make_unique<krosc::moveit::MoveGroup>(
        context_, "move_group",
        std::map<std::string, krosc::ParamValue>{
            {"base_link", std::string("base_link")},
            {"tip_link", std::string("end_effector_link")},
            {"joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4"}},
            {"lower_limits", std::vector<double>{-3.141592653589793, -2.20, -2.00, -2.00}},
            {"upper_limits", std::vector<double>{3.141592653589793, 2.20, 2.00, 2.20}},
            {"max_joint_speed", 0.35},
            {"max_joint_accel", 1.0},
            {"cartesian_speed", 0.08},
            {"cartesian_accel", 0.25},
            {"sample_time", 0.05},
            {"kinematics.plugin", std::string("krosc_moveit/ArmChainKinematics")},
            {"planner.plugin", std::string("krosc_moveit/CartesianPlanner")},
            {"controller_manager.plugin", std::string("krosc_moveit/SimpleControllerManager")},
            {"controller_manager.action",
             std::string("joint_trajectory_controller/follow_joint_trajectory")}});
  }

  if (options_.enable_map) {
    cartographer_ = std::make_unique<krosc::nodes::Cartographer>(
        context_, "cartographer_node",
        std::map<std::string, krosc::ParamValue>{
            {"mode", std::string(options_.slam ? "mapping" : "idle")},
            {"match_every", std::int64_t{1}},
            {"num_matching_points", std::int64_t{20}},
            {"linear_search_window", 0.25},
            {"angular_search_window", 15.0 * std::numbers::pi / 180.0},
            {"map_publish_period_sec", 2.0},
            {"tf_publish_period_sec", 0.02},
            {"tf_smoothing", 0.35}});
    map_server_ = std::make_unique<krosc::nodes::MapServer>(
        context_, "map_server",
        std::map<std::string, krosc::ParamValue>{{"maps_directory", options_.maps_dir}});
    map_saver_ = std::make_unique<krosc::nodes::MapSaver>(
        context_, "map_saver",
        std::map<std::string, krosc::ParamValue>{{"maps_directory", options_.maps_dir}});
    map_catalog_ = std::make_unique<krosc::nodes::MapCatalog>(
        context_, "map_catalog",
        std::map<std::string, krosc::ParamValue>{
            {"maps_directory", options_.maps_dir},
            {"map_server", std::string("map_server")},
            {"map_saver", std::string("map_saver")},
            {"list_period", 1.0}});
  }

  global_costmap_ = std::make_unique<krosc::nodes::nav2::Costmap2DROS>(
      context_, "global_costmap",
      std::map<std::string, krosc::ParamValue>{
          {"global_frame", std::string("map")},
          {"robot_base_frame", std::string("base_footprint")},
          {"plugins", std::vector<std::string>{"static_layer", "inflation_layer"}},
          {"resolution", 0.05},
          {"robot_radius", 0.22},
          {"inflation_radius", 0.55},
          {"cost_scaling_factor", 3.0}});
  local_costmap_ = std::make_unique<krosc::nodes::nav2::Costmap2DROS>(
      context_, "local_costmap",
      std::map<std::string, krosc::ParamValue>{
          {"global_frame", std::string("odom")},
          {"robot_base_frame", std::string("base_footprint")},
          {"plugins", std::vector<std::string>{"obstacle_layer", "inflation_layer"}},
          {"rolling_window", true},
          {"width", 3.0},
          {"height", 3.0},
          {"resolution", 0.05},
          {"update_frequency", 10.0},
          {"observation_sources", std::vector<std::string>{"scan"}},
          {"scan.topic", std::string("scan")},
          {"obstacle_max_range", 2.5},
          {"raytrace_max_range", 3.0},
          {"robot_radius", 0.22},
          {"inflation_radius", 0.35},
          {"cost_scaling_factor", 3.0}});

  planner_ = std::make_unique<krosc::nodes::nav2::PlannerServer>(
      context_, *global_costmap_, "planner_server",
      std::map<std::string, krosc::ParamValue>{
          {"planner_plugins", std::vector<std::string>{"GridBased", "Theta", "Smac2D",
                                                       "SmacHybrid"}},
          {"GridBased.plugin", std::string("nav2_navfn_planner::NavfnPlanner")},
          {"Theta.plugin", std::string("nav2_theta_star_planner::ThetaStarPlanner")},
          {"Smac2D.plugin", std::string("nav2_smac_planner::SmacPlanner2D")},
          {"SmacHybrid.plugin", std::string("nav2_smac_planner::SmacPlannerHybrid")},
          {"SmacHybrid.minimum_turning_radius", 0.2}});

  krosc::nodes::nav2::ControllerRegistry controller_registry =
      krosc::nodes::nav2::builtin_controllers();
  std::string follow_plugin =
      "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController";
#ifdef KROSC_WS_ENABLE_RL_NAV
  if (options_.enable_rl) {
    controller_registry.add("rl_nav_controller::RlNavController",
                            [] { return std::make_unique<krosc_ws::RlNavController>(); });
    follow_plugin = "rl_nav_controller::RlNavController";
  }
#endif
  controller_ = std::make_unique<krosc::nodes::nav2::ControllerServer>(
      context_, *local_costmap_, "controller_server",
      std::map<std::string, krosc::ParamValue>{
          {"controller_plugins", std::vector<std::string>{"FollowPath"}},
          {"FollowPath.plugin", follow_plugin},
          {"FollowPath.desired_linear_vel", 0.20},
          {"FollowPath.lookahead_dist", 0.35},
          {"FollowPath.use_collision_detection", true},
          {"FollowPath.rotate_to_heading_min_angle", 0.35},
          {"cmd_vel_topic", std::string("cmd_vel_nav")},
          {"controller_frequency", 50.0},
          {"general_goal_checker.xy_goal_tolerance", 0.20},
          {"general_goal_checker.yaw_goal_tolerance", 0.35},
          {"max_linear_accel", 0.0},
          {"max_angular_accel", 0.0}},
      controller_registry);
  behavior_ = std::make_unique<krosc::nodes::nav2::BehaviorServer>(
      context_, "behavior_server", std::map<std::string, krosc::ParamValue>{{"cmd_vel_topic", std::string("cmd_vel_nav")}});
  bt_navigator_ = std::make_unique<krosc::nodes::nav2::BtNavigator>(
      context_, "bt_navigator",
      std::map<std::string, krosc::ParamValue>{{"replanning_period", 1.0},
                                               {"number_of_recoveries", std::int64_t{3}},
                                               {"recovery_backup", true}});
  mux_ = std::make_unique<krosc::nodes::TwistMux>(
      context_, "twist_mux",
      std::map<std::string, krosc::ParamValue>{
          {"cmd_vel_out", std::string("cmd_vel_mux")},
          {"topics", std::vector<std::string>{"navigation", "teleop"}},
          {"navigation.topic", std::string("cmd_vel_nav")},
          {"navigation.priority", std::int64_t{10}},
          {"navigation.timeout", 0.3},
          {"teleop.topic", std::string("cmd_vel_teleop")},
          {"teleop.priority", std::int64_t{100}},
          {"teleop.timeout", 0.5}});
  smoother_ = std::make_unique<krosc::nodes::VelocitySmoother>(
      context_, "velocity_smoother",
      std::map<std::string, krosc::ParamValue>{
          {"cmd_vel_in_topic", std::string("cmd_vel_mux")},
          {"cmd_vel_out_topic", std::string("cmd_vel_smoothed")},
          {"max_linear_accel", 0.5},
          {"max_angular_accel", 2.0},
          {"max_linear_speed", 0.26},
          {"max_angular_speed", 1.82},
          {"frequency", 100.0},
          {"input_timeout", 0.5}});
  collision_ = std::make_unique<krosc::nodes::CollisionMonitor>(
      context_, "collision_monitor",
      std::map<std::string, krosc::ParamValue>{
          {"cmd_vel_in_topic", std::string("cmd_vel_smoothed")},
          {"cmd_vel_out_topic", std::string("cmd_vel")},
          {"PolygonSlow.slowdown_ratio", 0.6},
          {"ignore_collision_during_teleop", true}});

  sensors_ = std::make_unique<krosc::sensor::SensorManager>();
  (void)sensors_->add(*imu_, "opencr_imu", options_.imu_period);
  if (lds_sensor_) {
    (void)sensors_->add(*lds_sensor_, "lds03_scan", options_.scan_period);
  }
  if (camera_sensor_) {
    (void)sensors_->add(*camera_sensor_, "realsense", options_.camera_period);
  }
  controls_ = std::make_unique<krosc::control::ControlManager>();
  (void)controls_->add(*control_, "opencr_control", options_.control_read_period,
                       options_.control_write_period);
  controls_->set_control_hook(
      [this](double time, double period) { controller_manager_->update(time, period); },
      options_.control_period);
}

Engine::~Engine() {
  stop();
  // start()가 실패해 started_가 false여도 카메라 스트림은 정리한다.
  if (camera_ != nullptr) {
    camera_->stop();
  }
}

bool Engine::start() {
  if (started_) {
    return false;
  }
  (void)sensors_->start();
  if (!controls_->start()) {
    sensors_->stop();
    return false;
  }
  started_ = true;
  // 연결은 reader 스레드가 비동기로 연다. 시작 시 한 번 상태를 찍어 두고,
  // 이후에는 RobotStatus diagnostics가 1초마다 갱신한다.
  const bool opencr_ok = opencr_.is_connected();
  const bool lidar_ok = lidar_ == nullptr || lidar_->is_connected();
  const bool camera_ok =
      camera_ == nullptr || !options_.enable_camera || camera_->is_connected();
  std::fprintf(stderr, "[krosc_ws] hardware: %s (opencr=%s lidar=%s camera=%s)\n",
               is_connected() ? "connected" : "waiting", opencr_ok ? "yes" : "no",
               lidar_ok ? "yes" : "no", camera_ok ? "yes" : "no");
  return true;
}

bool Engine::is_connected() const {
  if (!opencr_.is_connected()) {
    return false;
  }
  if (lidar_ != nullptr && !lidar_->is_connected()) {
    return false;
  }
  if (camera_ != nullptr && options_.enable_camera && !camera_->is_connected()) {
    return false;
  }
  return true;
}

void Engine::connect_hardware() {
  if (!opencr_.is_connected()) {
    opencr_.connect();
  }
  if (lidar_ != nullptr && !lidar_->is_connected()) {
    lidar_->connect();
  }
  if (camera_ != nullptr && options_.enable_camera && !camera_->is_connected()) {
    camera_->connect();
  }
}

void Engine::disconnect_hardware() {
  if (opencr_.is_connected()) {
    opencr_.disconnect();
  }
  if (lidar_ != nullptr && lidar_->is_connected()) {
    lidar_->disconnect();
  }
  if (camera_ != nullptr && options_.enable_camera && camera_->is_connected()) {
    camera_->disconnect();
  }
}

void Engine::stop() {
  if (!started_) {
    return;
  }
  controls_->stop();
  sensors_->stop();
  if (lidar_ != nullptr) {
    lidar_->stop();
  }
  if (camera_ != nullptr) {
    camera_->stop();
  }
  opencr_.stop();
  started_ = false;
}

void Engine::log_status() {
  const auto now = std::chrono::steady_clock::now();
  if (last_status_.time_since_epoch().count() != 0 &&
      now - last_status_ < std::chrono::seconds(5)) {
    return;
  }
  last_status_ = now;
  if (cartographer_) {
    std::printf("[krosc_ws] slam: mode=%s scans=%d rejected=%d localized=%s cells=%zu\n",
                cartographer_->mode().c_str(), cartographer_->scans(), cartographer_->rejected(),
                cartographer_->localized() ? "yes" : "no", cartographer_->map_cells());
    std::fflush(stdout);
  }
}

}  // namespace krosc_ws
