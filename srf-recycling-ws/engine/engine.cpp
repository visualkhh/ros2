#include "engine.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "srf/control/control_manager.hpp"
#include "srf/control/controller_manager.hpp"
#include "srf/nodes/cartographer.hpp"
#include "srf/nodes/collision_monitor.hpp"
#include "srf/nodes/ekf.hpp"
#include "srf/nodes/map_catalog.hpp"
#include "srf/nodes/map_saver.hpp"
#include "srf/nodes/map_server.hpp"
#include "srf/nodes/moveit/move_group.hpp"
#include "srf/nodes/nav2/behavior_server.hpp"
#include "srf/nodes/nav2/bt_navigator.hpp"
#include "srf/nodes/nav2/controller_server.hpp"
#include "srf/nodes/nav2/costmap/costmap_2d_ros.hpp"
#include "srf/nodes/nav2/planner_server.hpp"
#include "srf/nodes/robot_state_publisher.hpp"
#include "srf/nodes/twist_mux.hpp"
#include "srf/nodes/velocity_smoother.hpp"
#include "srf/msgs.hpp"
#include "srf/qos.hpp"
#include "srf/sensor/sensor_manager.hpp"
#include "srf/urdf.hpp"

#include "controls/opencr/opencr_control.hpp"
#include "hardwares/lds03/lds_device.hpp"
#include "hardwares/opencr/opencr_device.hpp"
#include "nodes/ai_context/ai_context_node.hpp"
#include "nodes/grasp_detector/grasp_detector_node.hpp"
#include "nodes/llm_input/llm_input.hpp"
#include "nodes/nav_behavior/nav_behavior_node.hpp"
#include "nodes/pick_place/pick_place_node.hpp"
#include "nodes/robot_status/robot_status.hpp"
#include "nodes/vla/vla_node.hpp"
#include "nodes/vlm/vlm_node.hpp"
#include "sensors/lds03/lds03_sensor.hpp"
#include "sensors/opencr/opencr_imu.hpp"
#include "sensors/realsense/realsense_sensor.hpp"

#include "hardwares/realsense/realsense_camera.hpp"

#ifdef SRF_RECYCLING_WS_ENABLE_RL_NAV
#include "nodes/rl_nav/rl_nav_controller.hpp"
#endif

namespace srf_recycling_ws {

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

// AI가 "내 몸 크기"를 하드코딩이 아니라 실제 URDF에서 알도록 정적 치수를 뽑는다.
// 실시간 카메라/TCP 위치는 AiContextNode가 TF로 따로 채운다.
RobotGeometry measure_robot_geometry(const std::string& text) {
  RobotGeometry geometry;
  srf::UrdfRobot robot;
  try {
    robot = srf::parse_urdf(text);
  } catch (const std::exception&) {
    return geometry;
  }

  std::map<std::string, const srf::UrdfJoint*> parent_joint;
  for (const auto& joint : robot.joints) {
    parent_joint[joint.child] = &joint;
  }

  const auto link_offset = [&parent_joint](const std::string& name) {
    std::array<double, 3> offset{0.0, 0.0, 0.0};
    std::string current = name;
    for (int depth = 0; depth < 32; ++depth) {
      const auto found = parent_joint.find(current);
      if (found == parent_joint.end()) {
        break;
      }
      const auto& joint = *found->second;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        offset[axis] += joint.xyz[axis];
      }
      current = joint.parent;
      if (current == "base_footprint") {
        break;
      }
    }
    return offset;
  };

  // 몸체(footprint/height)는 base_link 충돌 박스와 base_scan까지의 고정 체인에서 구한다.
  if (const auto* base = robot.find_link("base_link")) {
    for (const auto& collision : base->collisions) {
      if (collision.geometry.type != srf::UrdfGeometryType::Box) {
        continue;
      }
      geometry.footprint[0] = collision.geometry.size[0];
      geometry.footprint[1] = collision.geometry.size[1];
      geometry.footprint[2] = link_offset("base_link")[2] + collision.xyz[2] +
                              collision.geometry.size[2] * 0.5;
      break;
    }
  }
  for (const auto* link : {robot.find_link("base_scan"), robot.find_link("base_link")}) {
    if (link == nullptr) {
      continue;
    }
    for (const auto& collision : link->collisions) {
      if (collision.geometry.type != srf::UrdfGeometryType::Box) {
        continue;
      }
      const double top = link_offset(link->name)[2] + collision.xyz[2] +
                         collision.geometry.size[2] * 0.5;
      geometry.footprint[2] = std::max(geometry.footprint[2], top);
      break;
    }
  }

  // 팔 베이스와 완전히 폈을 때의 도달 거리.
  for (const auto& joint : robot.joints) {
    if (joint.name == "arm_base_joint") {
      geometry.arm_base = joint.xyz;
      break;
    }
  }
  double reach = 0.0;
  std::string current = "link1";
  for (int depth = 0; depth < 16; ++depth) {
    const srf::UrdfJoint* next = nullptr;
    for (const auto& joint : robot.joints) {
      if (joint.parent == current && joint.movable()) {
        next = &joint;
        break;
      }
    }
    if (next == nullptr) {
      break;
    }
    reach += std::hypot(std::hypot(next->xyz[0], next->xyz[1]), next->xyz[2]);
    current = next->child;
  }
  if (const auto* tip = robot.find_link("gripper_left_link")) {
    for (const auto& collision : tip->collisions) {
      if (collision.geometry.type == srf::UrdfGeometryType::Box) {
        reach += collision.xyz[0] + collision.geometry.size[0] * 0.5;
        break;
      }
    }
  }
  if (reach > 0.0) {
    geometry.arm_reach = reach;
  }
  geometry.from_urdf = true;
  return geometry;
}

}  // namespace

Engine::Engine(srf::Context& context, OpenCrDevice& opencr, LdsDevice* lidar,
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
  if (options_.enable_llm) {
    llm_ = std::make_unique<LlmInput>(context_, options_.llm_ollama_url,
                                      options_.llm_model, options_.llm_timeout,
                                      options_.llm_mode, options_.camera_color_topic,
                                      options_.llm_include_image);
  }
  if (options_.enable_vla) {
    vla_ = std::make_unique<VlaNode>(context_, options_.vla_url,
                                     options_.camera_color_topic,
                                     options_.vla_period, options_.vla_timeout,
                                     options_.vla_replan_steps);
  }
  if (options_.enable_vlm) {
    vlm_ = std::make_unique<VlmNode>(context_, options_.vlm_url,
                                     options_.camera_color_topic,
                                     options_.vlm_timeout);
  }
  if (options_.enable_grasp_detector) {
    grasp_detector_ = std::make_unique<GraspDetectorNode>(
        context_, options_.camera_color_topic, options_.camera_depth_topic,
        options_.pick_objects_topic, options_.pick_grasp_info_topic);
  }
  if (options_.enable_ai_context) {
    AiServerConfig ai_servers;
    ai_servers.llm_enabled = options_.enable_llm;
    ai_servers.llm_url = options_.llm_ollama_url;
    ai_servers.llm_model = options_.llm_model;
    ai_servers.vlm_enabled = options_.enable_vlm;
    ai_servers.vlm_url = options_.vlm_url;
    ai_servers.vla_enabled = options_.enable_vla;
    ai_servers.vla_url = options_.vla_url;
    ai_context_ = std::make_unique<AiContextNode>(
        context_, options_.scan_topic, options_.camera_color_topic,
        options_.camera_depth_topic, measure_robot_geometry(urdf_),
        std::move(ai_servers));
  }
  if (options_.enable_pick) {
    pick_ = std::make_unique<PickPlaceNode>(context_, options_.pick_objects_topic,
                                            options_.pick_grasp_info_topic);
  }
  if (options_.enable_nav_behavior) {
    nav_behavior_ = std::make_unique<NavBehaviorNode>(
        context_, options_.nav_costmap_topic, options_.nav_scan_topic);
  }

  std::vector<std::string> controller_plugins{"joint_state_broadcaster", "diff_drive_controller"};
  if (options_.enable_arm) {
    controller_plugins.push_back("joint_trajectory_controller");
    controller_plugins.push_back("arm_command_controller");
  }
  std::map<std::string, srf::ParamValue> controller_params{
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
  controller_manager_ = std::make_unique<srf::control::ControllerManager>(
      context_, "controller_manager", controller_params, *control_);

  ekf_ = std::make_unique<srf::nodes::Ekf>(
      context_, "ekf_filter_node",
      std::map<std::string, srf::ParamValue>{
          {"odom0", std::string("odom")},
          {"odom0_config", ekf_config({5, 6, 11})},
          {"frequency", 50.0},
          {"publish_tf", true}});

  if (options_.enable_robot_state_publisher) {
    robot_state_ = std::make_unique<srf::nodes::RobotStatePublisher>(
        context_, "robot_state_publisher",
        std::map<std::string, srf::ParamValue>{{"robot_description", urdf_}});
  }

  if (options_.enable_moveit) {
    move_group_ = std::make_unique<srf::moveit::MoveGroup>(
        context_, "move_group",
        std::map<std::string, srf::ParamValue>{
            {"base_link", std::string("base_link")},
            {"tip_link", std::string("end_effector_link")},
            {"joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4"}},
            {"lower_limits", std::vector<double>{-2.827433388230814, -1.790707812546182, -0.9424777960769379, -1.790707812546182}},
            {"upper_limits", std::vector<double>{2.827433388230814, 1.5707963267948966, 1.382300767579509, 2.0420352248333655}},
            {"max_joint_speed", 0.35},
            {"max_joint_accel", 1.0},
            {"cartesian_speed", 0.08},
            {"cartesian_accel", 0.25},
            {"sample_time", 0.05},
            {"kinematics.plugin", std::string("srf_moveit/ArmChainKinematics")},
            {"planner.plugin", std::string("srf_moveit/CartesianPlanner")},
            {"controller_manager.plugin", std::string("srf_moveit/SimpleControllerManager")},
            {"controller_manager.action",
             std::string("joint_trajectory_controller/follow_joint_trajectory")}});
  }

  if (options_.enable_map) {
    cartographer_ = std::make_unique<srf::nodes::Cartographer>(
        context_, "cartographer_node",
        std::map<std::string, srf::ParamValue>{
            {"mode", std::string(options_.slam ? "mapping" : "idle")},
            {"match_every", std::int64_t{1}},
            {"num_matching_points", std::int64_t{20}},
            {"linear_search_window", 0.25},
            {"angular_search_window", 15.0 * std::numbers::pi / 180.0},
            {"map_publish_period_sec", 2.0},
            {"tf_publish_period_sec", 0.02},
            {"tf_smoothing", 0.35}});
    map_server_ = std::make_unique<srf::nodes::MapServer>(
        context_, "map_server",
        std::map<std::string, srf::ParamValue>{{"maps_directory", options_.maps_dir}});
    map_saver_ = std::make_unique<srf::nodes::MapSaver>(
        context_, "map_saver",
        std::map<std::string, srf::ParamValue>{{"maps_directory", options_.maps_dir}});
    map_catalog_ = std::make_unique<srf::nodes::MapCatalog>(
        context_, "map_catalog",
        std::map<std::string, srf::ParamValue>{
            {"maps_directory", options_.maps_dir},
            {"map_server", std::string("map_server")},
            {"map_saver", std::string("map_saver")},
            {"list_period", 1.0}});
  }

  global_costmap_ = std::make_unique<srf::nodes::nav2::Costmap2DROS>(
      context_, "global_costmap",
      std::map<std::string, srf::ParamValue>{
          {"global_frame", std::string("map")},
          {"robot_base_frame", std::string("base_footprint")},
          {"plugins", std::vector<std::string>{"static_layer", "inflation_layer"}},
          {"resolution", 0.05},
          {"robot_radius", 0.22},
          {"inflation_radius", 0.55},
          {"cost_scaling_factor", 3.0}});
  local_costmap_ = std::make_unique<srf::nodes::nav2::Costmap2DROS>(
      context_, "local_costmap",
      std::map<std::string, srf::ParamValue>{
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

  planner_ = std::make_unique<srf::nodes::nav2::PlannerServer>(
      context_, *global_costmap_, "planner_server",
      std::map<std::string, srf::ParamValue>{
          {"planner_plugins", std::vector<std::string>{"GridBased", "Theta", "Smac2D",
                                                       "SmacHybrid"}},
          {"GridBased.plugin", std::string("nav2_navfn_planner::NavfnPlanner")},
          {"Theta.plugin", std::string("nav2_theta_star_planner::ThetaStarPlanner")},
          {"Smac2D.plugin", std::string("nav2_smac_planner::SmacPlanner2D")},
          {"SmacHybrid.plugin", std::string("nav2_smac_planner::SmacPlannerHybrid")},
          {"SmacHybrid.minimum_turning_radius", 0.2}});

  srf::nodes::nav2::ControllerRegistry controller_registry =
      srf::nodes::nav2::builtin_controllers();
  std::string follow_plugin =
      "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController";
#ifdef SRF_RECYCLING_WS_ENABLE_RL_NAV
  if (options_.enable_rl) {
    controller_registry.add("rl_nav_controller::RlNavController",
                            [] { return std::make_unique<srf_recycling_ws::RlNavController>(); });
    follow_plugin = "rl_nav_controller::RlNavController";
  }
#endif
  controller_ = std::make_unique<srf::nodes::nav2::ControllerServer>(
      context_, *local_costmap_, "controller_server",
      std::map<std::string, srf::ParamValue>{
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
  behavior_ = std::make_unique<srf::nodes::nav2::BehaviorServer>(
      context_, "behavior_server", std::map<std::string, srf::ParamValue>{{"cmd_vel_topic", std::string("cmd_vel_nav")}});
  bt_navigator_ = std::make_unique<srf::nodes::nav2::BtNavigator>(
      context_, "bt_navigator",
      std::map<std::string, srf::ParamValue>{{"replanning_period", 1.0},
                                               {"number_of_recoveries", std::int64_t{3}},
                                               {"recovery_backup", true}});
  mux_ = std::make_unique<srf::nodes::TwistMux>(
      context_, "twist_mux",
      std::map<std::string, srf::ParamValue>{
          {"cmd_vel_out", std::string("cmd_vel_mux")},
          {"topics", std::vector<std::string>{"navigation", "teleop"}},
          {"navigation.topic", std::string("cmd_vel_nav")},
          {"navigation.priority", std::int64_t{10}},
          {"navigation.timeout", 0.3},
          {"teleop.topic", std::string("cmd_vel_teleop")},
          {"teleop.priority", std::int64_t{100}},
          {"teleop.timeout", 0.5}});
  smoother_ = std::make_unique<srf::nodes::VelocitySmoother>(
      context_, "velocity_smoother",
      std::map<std::string, srf::ParamValue>{
          {"cmd_vel_in_topic", std::string("cmd_vel_mux")},
          {"cmd_vel_out_topic", std::string("cmd_vel_smoothed")},
          {"max_linear_accel", 0.5},
          {"max_angular_accel", 2.0},
          {"max_linear_speed", 0.26},
          {"max_angular_speed", 1.82},
          {"frequency", 100.0},
          {"input_timeout", 0.5}});
  collision_ = std::make_unique<srf::nodes::CollisionMonitor>(
      context_, "collision_monitor",
      std::map<std::string, srf::ParamValue>{
          {"cmd_vel_in_topic", std::string("cmd_vel_smoothed")},
          {"cmd_vel_out_topic", std::string("cmd_vel")},
          {"PolygonSlow.slowdown_ratio", 0.6},
          {"ignore_collision_during_teleop", true}});

  sensors_ = std::make_unique<srf::sensor::SensorManager>();
  (void)sensors_->add(*imu_, "opencr_imu", options_.imu_period);
  if (lds_sensor_) {
    (void)sensors_->add(*lds_sensor_, "lds03_scan", options_.scan_period);
  }
  if (camera_sensor_) {
    (void)sensors_->add(*camera_sensor_, "realsense", options_.camera_period);
  }
  controls_ = std::make_unique<srf::control::ControlManager>();
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
  std::fprintf(stderr, "[srf_recycling_ws] hardware: %s (opencr=%s lidar=%s camera=%s)\n",
               is_connected() ? "connected" : "waiting", opencr_ok ? "yes" : "no",
               lidar_ok ? "yes" : "no", camera_ok ? "yes" : "no");
  publish_ai_info();
  return true;
}

void Engine::publish_ai_info() {
  // AI 패널이 "어느 서버/모델로 배선됐는지" 바로 보여줄 수 있게 latched JSON.
  std::ostringstream out;
  const std::string llm_model = options_.llm_model.empty()
                                    ? "gateway"
                                    : options_.llm_model;
  out << "{\"llm\":{\"enabled\":" << (options_.enable_llm ? "true" : "false")
      << ",\"url\":\"" << options_.llm_ollama_url << "\",\"model\":\""
      << llm_model << "\",\"mode\":\"" << options_.llm_mode
      << "\",\"image\":" << (options_.llm_include_image ? "true" : "false")
      << "},"
      << "\"vlm\":{\"enabled\":" << (options_.enable_vlm ? "true" : "false")
      << ",\"url\":\"" << options_.vlm_url << "\",\"model\":\""
      << options_.vlm_model << "\"},"
      << "\"vla\":{\"enabled\":" << (options_.enable_vla ? "true" : "false")
      << ",\"url\":\"" << options_.vla_url << "\",\"model\":\""
      << options_.vla_model << "\"},"
      << "\"grasp\":{\"enabled\":" << (options_.enable_grasp_detector ? "true" : "false")
      << ",\"source\":\"camera/color+depth\"},"
      << "\"services\":{\"command\":\"llm/command\",\"pick\":\"pick/command\","
      << "\"nav\":\"nav/hide|nav/approach\",\"vlm\":\"vlm/query\",\"vla\":\"vla/command\"}}";
  context_.topics().publish<srf::msgs::String>(
      "ai_info", srf::msgs::String{out.str()}, srf::QOS_LATCHED);
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
  if (llm_) {
    llm_->emergency_stop();
  }
  if (vla_) {
    vla_->emergency_stop();
  }
  if (pick_) {
    pick_->emergency_stop();
  }
  if (nav_behavior_) {
    nav_behavior_->emergency_stop();
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
  // latched만 보내면 gateway 재접속 후 브라우저가 못 받을 수 있다. 5초마다 갱신.
  publish_ai_info();
  if (cartographer_) {
    std::printf("[srf_recycling_ws] slam: mode=%s scans=%d rejected=%d localized=%s cells=%zu\n",
                cartographer_->mode().c_str(), cartographer_->scans(), cartographer_->rejected(),
                cartographer_->localized() ? "yes" : "no", cartographer_->map_cells());
    std::fflush(stdout);
  }
}

}  // namespace srf_recycling_ws
