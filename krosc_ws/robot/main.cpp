#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/msgs.hpp"
#include "krosc/nodes/moveit/move_group.hpp"
#include "krosc/net/net.hpp"
#include "krosc/qos.hpp"
#include "krosc/nodes/cartographer.hpp"
#include "krosc/nodes/collision_monitor.hpp"
#include "krosc/nodes/ekf.hpp"
#include "krosc/nodes/map_catalog.hpp"
#include "krosc/nodes/map_saver.hpp"
#include "krosc/nodes/map_server.hpp"
#include "krosc/nodes/nav2/behavior_server.hpp"
#include "krosc/nodes/nav2/bt_navigator.hpp"
#include "krosc/nodes/nav2/controller_server.hpp"
#include "krosc/nodes/nav2/costmap/costmap_2d_ros.hpp"
#include "krosc/nodes/nav2/planner_server.hpp"
#include "krosc/nodes/robot_state_publisher.hpp"
#include "krosc/nodes/twist_mux.hpp"
#include "krosc/nodes/velocity_smoother.hpp"
#include "krosc/control/controller_manager.hpp"
#include "krosc/time/time.hpp"

#include "hardwares/controls/opencr/opencr_system.hpp"
#include "real_turtlebot3.hpp"
#include "turtlebot3_driver.hpp"

#ifdef KROSC_WS_ENABLE_RL_NAV
#include "nodes/rl_nav_controller.hpp"
#endif

#ifndef KROSC_WS_URDF_PATH
#define KROSC_WS_URDF_PATH "robot/description/urdf/turtlebot3_real.urdf"
#endif
#ifndef KROSC_WS_MAPS_DIR
#define KROSC_WS_MAPS_DIR "maps"
#endif

namespace {

std::atomic<bool> g_stop{false};

void request_stop(int) { g_stop.store(true); }

std::string env_or(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  return value == nullptr ? fallback : std::string(value);
}

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
  for (int index : enabled) {
    if (index >= 0 && index < 15) {
      config[static_cast<std::size_t>(index)] = true;
    }
  }
  return config;
}

std::vector<std::string> local_addresses() {
  std::vector<std::string> result;
  ifaddrs* addresses = nullptr;
  if (::getifaddrs(&addresses) != 0) {
    return result;
  }
  for (ifaddrs* item = addresses; item != nullptr; item = item->ifa_next) {
    if (item->ifa_addr == nullptr || item->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    const auto* address = reinterpret_cast<const sockaddr_in*>(item->ifa_addr);
    const std::uint32_t host = ntohl(address->sin_addr.s_addr);
    if ((host >> 24) == 127) {
      continue;
    }
    char text[INET_ADDRSTRLEN];
    if (::inet_ntop(AF_INET, &address->sin_addr, text, sizeof(text)) != nullptr) {
      result.emplace_back(text);
    }
  }
  ::freeifaddrs(addresses);
  return result;
}

void usage() {
  std::cout << "usage: krosc_real [--opencr PORT] [--lidar PORT] [--port PORT] [--urdf PATH] [--maps DIR] [--no-lidar] [--slam]\n"
               "  --slam: SLAM(mapping)을 켠다 (없으면 idle). 원격 viz는 --fixed-frame map\n"
               "env: OPENCR_PORT (/dev/opencr), LIDAR_PORT (auto), KROSC_SERVER_PORT (9000), KROSC_MAPS_DIR\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string opencr_port = env_or("OPENCR_PORT", "/dev/opencr");
  std::string lidar_port = env_or("LIDAR_PORT", "");
  std::string urdf_path = KROSC_WS_URDF_PATH;
  std::string maps_dir = env_or("KROSC_MAPS_DIR", KROSC_WS_MAPS_DIR);
  int server_port = 9000;
  {
    const std::string port_env = env_or("KROSC_SERVER_PORT", "9000");
    try {
      server_port = std::stoi(port_env);
    } catch (...) {
      server_port = 9000;
    }
  }
  bool enable_lidar = true;
  bool slam = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--opencr" && i + 1 < argc) {
      opencr_port = argv[++i];
    } else if (argument == "--lidar" && i + 1 < argc) {
      lidar_port = argv[++i];
    } else if (argument == "--port" && i + 1 < argc) {
      server_port = std::stoi(argv[++i]);
    } else if (argument == "--urdf" && i + 1 < argc) {
      urdf_path = argv[++i];
    } else if (argument == "--maps" && i + 1 < argc) {
      maps_dir = argv[++i];
    } else if (argument == "--no-lidar") {
      enable_lidar = false;
    } else if (argument == "--slam") {
      slam = true;
    } else if (argument == "-h" || argument == "--help") {
      usage();
      return 0;
    } else {
      std::cerr << "unknown argument: " << argument << "\n";
      usage();
      return 2;
    }
  }

  const std::string urdf = read_text(urdf_path);
  if (urdf.empty()) {
    std::cerr << "[krosc_ws] URDF를 읽지 못했다: " << urdf_path << "\n";
    return 1;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  krosc::Context::Options options;
  options.name = "krosc_ws";
  options.time_source = std::make_shared<krosc::SystemTimeSource>();
  if (server_port > 0) {
    options.network.name = "krosc_ws";
    options.network.host = "0.0.0.0";
    options.network.port = static_cast<std::uint16_t>(server_port);
    // Viz의 텔레옵은 cmd_vel로 오지만 로봇에서는 cmd_vel_teleop으로 받아야 한다.
    options.network.receive_remap["cmd_vel"] = "cmd_vel_teleop";
  }
  krosc::Context context(options);

  if (const auto* peer = context.peer_network()) {
    const int port = static_cast<int>(peer->port());
    std::printf("[krosc_ws] peer listen: 0.0.0.0:%d\n", port);
    const auto addresses = local_addresses();
    if (addresses.empty()) {
      std::printf("[krosc_ws] peer: <pi-ip>:%d\n", port);
    } else {
      for (const auto& address : addresses) {
        std::printf("[krosc_ws] peer: %s:%d\n", address.c_str(), port);
      }
    }
    std::printf("[krosc_ws] Viz: --server <ip>:%d\n", port);
    std::fflush(stdout);
  }

  krosc_ws::RealTurtleBot3 robot(opencr_port, lidar_port, enable_lidar);
  krosc_ws::TurtleBot3Driver driver(context, robot);
  krosc::nodes::RobotStatePublisher robot_state(context, "robot_state_publisher",
                                                {{"robot_description", urdf}});
  // control: OpenCR을 SystemInterface 플러그인으로 올리고, controller_manager가 JSB/JTC를 돌린다.
  // joint_states는 이제 joint_state_broadcaster가 내고, MoveIt은 JTC의 FollowJointTrajectory 액션으로 움직인다.
  krosc::control::HardwareRegistry hardware_registry;
  hardware_registry.add("krosc_ws/OpenCrSystem",
                        [&robot] { return std::make_unique<krosc_ws::OpenCrSystem>(robot); });
  krosc::control::ControllerManager controller_manager(
      context, "controller_manager",
      {{"update_rate", 100.0},
       {"hardware.plugin", std::string("krosc_ws/OpenCrSystem")},
       {"hardware.name", std::string("turtlebot3_real")},
       {"controller_plugins",
        std::vector<std::string>{"joint_state_broadcaster", "joint_trajectory_controller",
                                 "diff_drive_controller", "arm_command_controller"}},
       {"joint_state_broadcaster.type",
        std::string("joint_state_broadcaster/JointStateBroadcaster")},
       {"joint_trajectory_controller.type",
        std::string("joint_trajectory_controller/JointTrajectoryController")},
       {"joint_trajectory_controller.joints",
        std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "gripper_left_joint"}},
       {"joint_trajectory_controller.goal_time_tolerance", 0.5},
       {"diff_drive_controller.type", std::string("diff_drive_controller/DiffDriveController")},
       {"diff_drive_controller.wheel_separation", 0.287},
       {"diff_drive_controller.wheel_radius", 0.033},
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
       {"arm_command_controller.auto_activate", false}},
      hardware_registry);
  // MoveIt move_group: kinematics/planner/controller_manager 플러그인을 골라 조립한다.
  // Viz 명령은 move_action 액션 + move_group/scrub 서비스로 들어온다.
  krosc::moveit::MoveGroup move_group(
      context, "move_group",
      {{"base_link", std::string("base_link")},
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
  // IMU yaw rate는 정지 중에도 bias/진동이 커서 EKF에서 제외한다. 휠 오도메트리의
  // yaw 자세(5)를 절대 기준으로 쓰고, vx/vyaw(6/11)만 융합한다.
  krosc::nodes::Ekf ekf(context, "ekf_filter_node",
                        {{"odom0", std::string("odom")},
                         {"odom0_config", ekf_config({5, 6, 11})},
                         {"frequency", 50.0},
                         {"publish_tf", true}});
  constexpr double kDegreesToRadians = std::numbers::pi / 180.0;
  krosc::nodes::Cartographer cartographer(
      context, "cartographer_node",
      {{"mode", std::string(slam ? "mapping" : "idle")},
       {"match_every", std::int64_t{1}},
       {"num_matching_points", std::int64_t{20}},
       // 직진 0.26 m/s에서 매칭 간격(2스캔=0.2s) 동안 약 5cm 이동한다.
       // 기존 ±5cm/±3° 창은 이동량을 못 따라가 매칭이 자주 기각됐다(localization 드리프트).
       {"linear_search_window", 0.25},
       {"angular_search_window", 15.0 * kDegreesToRadians},
       {"map_publish_period_sec", 2.0},
       {"tf_publish_period_sec", 0.02},
       {"tf_smoothing", 0.35}});
  krosc::nodes::MapServer map_server(
      context, "map_server",
      {{"maps_directory", maps_dir}, {"topic_name", std::string("map")}, {"frame_id", std::string("map")}});
  krosc::nodes::MapSaver map_saver(context, "map_saver", {{"maps_directory", maps_dir}});
  krosc::nodes::MapCatalog maps(context, "map_catalog",
                                {{"maps_directory", maps_dir},
                                 {"map_frame", std::string("map")},
                                 {"map_server", std::string("map_server")},
                                 {"map_saver", std::string("map_saver")},
                                 {"list_period", 1.0}});
  // nav2처럼 내비게이션을 노드로 나눈다: costmap_2d -> planner_server -> controller_server -> bt_navigator.
  // Viz/웹은 nav2 표준 액션(navigate_to_pose, compute_path_to_pose, navigate_through_poses,
  // compute_path_through_poses)으로 명령한다. 상태 토픽(plan, goal_status, cmd_vel_nav)만 유지한다.
  krosc::nodes::nav2::Costmap2DROS global_costmap(
      context, "global_costmap",
      {{"global_frame", std::string("map")},
       {"robot_base_frame", std::string("base_footprint")},
       {"map_topic", std::string("map")},
       {"plugins", std::vector<std::string>{"static_layer", "inflation_layer"}},
       {"track_unknown_space", true},
       {"robot_radius", 0.22},
       {"inflation_radius", 0.55},
       {"cost_scaling_factor", 3.0}});
  krosc::nodes::nav2::Costmap2DROS local_costmap(
      context, "local_costmap",
      {{"global_frame", std::string("odom")},
       {"robot_base_frame", std::string("base_footprint")},
       {"plugins", std::vector<std::string>{"obstacle_layer", "inflation_layer"}},
       {"rolling_window", true},
       {"width", 4.0},
       {"height", 4.0},
       {"resolution", 0.05},
       {"update_frequency", 10.0},
       {"observation_sources", std::vector<std::string>{"scan"}},
       {"scan.topic", std::string("scan")},
       {"obstacle_max_range", 2.5},
       {"raytrace_max_range", 3.0},
       {"robot_radius", 0.22},
       {"inflation_radius", 0.55},
       {"cost_scaling_factor", 3.0}});
  krosc::nodes::nav2::PlannerServer planner_server(
      context, global_costmap, "planner_server",
      {{"planner_plugins", std::vector<std::string>{"GridBased"}},
       {"GridBased.plugin", std::string("nav2_navfn_planner::NavfnPlanner")},
       {"GridBased.tolerance", 0.5},
       {"GridBased.use_astar", true},
       {"GridBased.allow_unknown", false}});
  krosc::nodes::nav2::ControllerRegistry controller_registry =
      krosc::nodes::nav2::builtin_controllers();
#ifdef KROSC_WS_ENABLE_RL_NAV
  controller_registry.add("rl_nav_controller::RlNavController",
                          [] { return std::make_unique<krosc_ws::RlNavController>(); });
#endif
  const std::string follow_plugin =
#ifdef KROSC_WS_ENABLE_RL_NAV
      "rl_nav_controller::RlNavController";
#else
      "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController";
#endif
  krosc::nodes::nav2::ControllerServer controller_server(
      context, local_costmap, "controller_server",
      {{"controller_plugins", std::vector<std::string>{"FollowPath"}},
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
  krosc::nodes::nav2::BehaviorServer behavior_server(
      context, "behavior_server", {{"cmd_vel_topic", std::string("cmd_vel_nav")}});
  krosc::nodes::nav2::BtNavigator bt_navigator(
      context, "bt_navigator",
      {{"replanning_period", 1.0},
       {"number_of_recoveries", std::int64_t{3}},
       {"recovery_backup", true}});
  krosc::nodes::TwistMux mux(
      context, "twist_mux",
      {{"cmd_vel_out", std::string("cmd_vel_mux")},
       {"topics", std::vector<std::string>{"navigation", "teleop"}},
       {"navigation.topic", std::string("cmd_vel_nav")},
       {"navigation.priority", std::int64_t{10}},
       {"navigation.timeout", 0.3},
       {"teleop.topic", std::string("cmd_vel_teleop")},
       {"teleop.priority", std::int64_t{100}},
       {"teleop.timeout", 0.5}});
  krosc::nodes::VelocitySmoother smoother(
      context, "velocity_smoother",
      {{"cmd_vel_in_topic", std::string("cmd_vel_mux")},
       {"cmd_vel_out_topic", std::string("cmd_vel_smoothed")},
       {"max_linear_accel", 0.5},
       {"max_angular_accel", 2.0},
       {"max_linear_speed", 0.26},
       {"max_angular_speed", 1.82},
       {"frequency", 100.0},
       {"input_timeout", 0.5}});
  krosc::nodes::CollisionMonitor collision_monitor(
      context, "collision_monitor",
      {{"cmd_vel_in_topic", std::string("cmd_vel_smoothed")},
       {"cmd_vel_out_topic", std::string("cmd_vel")},
       {"PolygonSlow.slowdown_ratio", 0.6},
       {"ignore_collision_during_teleop", true}});
  std::printf("[krosc_ws] SLAM: %s\n", slam ? "mapping" : "idle");
  std::printf("[krosc_ws] maps: %s\n", maps_dir.c_str());

  std::printf("[krosc_ws] 실행. Ctrl+C로 정지한다.\n");
  std::fflush(stdout);
  constexpr auto kPeriod = std::chrono::milliseconds(10);
  constexpr double kPeriodSeconds = 0.01;
  auto next = std::chrono::steady_clock::now();
  auto last_status = std::chrono::steady_clock::now();
  while (!g_stop.load()) {
    robot.tick(context.now(), kPeriodSeconds);
    driver.poll();
    next += kPeriod;
    std::this_thread::sleep_until(next);
    const auto now = std::chrono::steady_clock::now();
    if (now - next > std::chrono::milliseconds(250)) {
      next = now;
    }
    if (now - last_status >= std::chrono::seconds(5)) {
      last_status = now;
      std::printf("[krosc_ws] slam: mode=%s scans=%d rejected=%d localized=%s cells=%zu\n",
                  cartographer.mode().c_str(), cartographer.scans(), cartographer.rejected(),
                  cartographer.localized() ? "yes" : "no", cartographer.map_cells());
      std::fflush(stdout);
    }
  }

  std::printf("[krosc_ws] 정지 중...\n");
  std::fflush(stdout);
  robot.stop();
  context.shutdown();
  std::printf("[krosc_ws] 종료\n");
  return 0;
}
