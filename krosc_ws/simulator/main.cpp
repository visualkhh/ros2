#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/control/control_manager.hpp"
#include "krosc/control/controller_manager.hpp"
#include "krosc/nodes/ekf.hpp"
#include "krosc/nodes/nav2/behavior_server.hpp"
#include "krosc/nodes/nav2/bt_navigator.hpp"
#include "krosc/nodes/nav2/controller_server.hpp"
#include "krosc/nodes/nav2/costmap/costmap_2d_ros.hpp"
#include "krosc/nodes/nav2/planner_server.hpp"
#include "krosc/nodes/velocity_smoother.hpp"
#include "krosc/nodes/collision_monitor.hpp"
#include "krosc/nodes/twist_mux.hpp"
#include "krosc/sensor/sensor_manager.hpp"
#include "krosc/simulator/simulator.hpp"
#include "krosc/time/time.hpp"

#include "sim_control.hpp"
#include "sim_sensors.hpp"

#ifndef KROSC_WS_SIM_URDF
#define KROSC_WS_SIM_URDF "turtlebot3_waffle_pi.urdf"
#endif

namespace {

std::atomic<bool> g_stop{false};
void request_stop(int) { g_stop.store(true); }

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

int main(int argc, char** argv) {
  int port = 9000;
  double duration = 0.0;
  int pucks = 10;
  std::uint32_t seed = 42;
  bool noisy = true;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) {
      port = std::stoi(argv[++i]);
    } else if (arg == "--duration" && i + 1 < argc) {
      duration = std::stod(argv[++i]);
    } else if (arg == "--pucks" && i + 1 < argc) {
      pucks = std::stoi(argv[++i]);
    } else if (arg == "--seed" && i + 1 < argc) {
      seed = static_cast<std::uint32_t>(std::stoul(argv[++i]));
    } else if (arg == "--no-noise") {
      noisy = false;
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "usage: krosc_ws_simulator [--port 9000] [--duration SEC] [--pucks N]\n"
                   "                          [--seed N] [--no-noise]\n";
      return 0;
    }
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  krosc::Context::Options context_options;
  context_options.name = "krosc_ws_sim";
  context_options.executor_threads = 10;
  context_options.time_source = std::make_shared<krosc::SimulatedTimeSource>();
  if (port > 0) {
    context_options.network.name = "krosc_ws_sim";
    context_options.network.host = "0.0.0.0";
    context_options.network.port = static_cast<std::uint16_t>(port);
    context_options.network.receive_remap["cmd_vel"] = "cmd_vel_teleop";
  }
  krosc::Context context(context_options);

  krosc::simulator::SimulatorOptions sim_options;
  sim_options.noisy = noisy;
  sim_options.pucks = pucks;
  sim_options.seed = seed;
  sim_options.step_hz = 100.0;
  sim_options.urdf_path = KROSC_WS_SIM_URDF;
  sim_options.bridge = false;
  sim_options.odometry = false;
  sim_options.ekf = false;
  krosc::simulator::Simulator simulator(context, sim_options);
  auto& robot = simulator.robot();

  krosc_ws::sim::SimControl control(robot, krosc::simulator::TurtleBot3::kWheelRadius,
                                    krosc::simulator::TurtleBot3::kWheelSeparation);
  krosc_ws::sim::SimImuSensor imu(context, robot, "imu", "imu_link");
  krosc_ws::sim::SimScanSensor scan(context, robot, "scan", "base_scan");

  krosc::control::ControllerManager controller_manager(
      context, "controller_manager",
      {{"update_rate", 100.0},
       {"controller_plugins", std::vector<std::string>{"joint_state_broadcaster",
                                                       "diff_drive_controller"}},
       {"joint_state_broadcaster.type",
        std::string("joint_state_broadcaster/JointStateBroadcaster")},
       {"diff_drive_controller.type", std::string("diff_drive_controller/DiffDriveController")},
       {"diff_drive_controller.wheel_separation",
        krosc::simulator::TurtleBot3::kWheelSeparation},
       {"diff_drive_controller.wheel_radius", krosc::simulator::TurtleBot3::kWheelRadius},
       {"diff_drive_controller.left_wheel", std::string("wheel_left_joint")},
       {"diff_drive_controller.right_wheel", std::string("wheel_right_joint")},
       {"diff_drive_controller.cmd_vel_topic", std::string("cmd_vel")},
       {"diff_drive_controller.odom_topic", std::string("odom")},
       {"diff_drive_controller.odom_frame", std::string("odom")},
       {"diff_drive_controller.base_frame", std::string("base_footprint")},
       {"diff_drive_controller.publish_tf", false},
       {"diff_drive_controller.command_timeout", 0.5},
       {"diff_drive_controller.odom_publish_rate", 50.0}},
      control);

  krosc::nodes::Ekf ekf(context, "ekf_filter_node",
                        {{"odom0", std::string("odom")},
                         {"odom0_config", ekf_config({6, 11})},
                         {"imu0", std::string("imu")},
                         {"imu0_config", ekf_config({11})},
                         {"frequency", 50.0},
                         {"publish_tf", true}});

  krosc::nodes::nav2::Costmap2DROS global(
      context, "global_costmap",
      {{"global_frame", "map"},
       {"robot_base_frame", "base_footprint"},
       {"plugins", std::vector<std::string>{"static_layer", "inflation_layer"}},
       {"resolution", 0.05},
       {"robot_radius", 0.22},
       {"inflation_radius", 0.35},
       {"cost_scaling_factor", 3.0}});
  krosc::nodes::nav2::Costmap2DROS local(
      context, "local_costmap",
      {{"global_frame", "odom"},
       {"robot_base_frame", "base_footprint"},
       {"plugins", std::vector<std::string>{"obstacle_layer", "inflation_layer"}},
       {"rolling_window", true},
       {"width", 3.0},
       {"height", 3.0},
       {"resolution", 0.05},
       {"obstacle_max_range", 2.5},
       {"raytrace_max_range", 3.0},
       {"robot_radius", 0.22},
       {"inflation_radius", 0.25},
       {"cost_scaling_factor", 3.0}});
  krosc::nodes::nav2::PlannerServer planner(
      context, global, "planner_server",
      {{"planner_plugins", std::vector<std::string>{"GridBased"}},
       {"GridBased.plugin", std::string("nav2_navfn_planner::NavfnPlanner")}});
  krosc::nodes::nav2::ControllerServer controller(
      context, local, "controller_server",
      {{"controller_plugins", std::vector<std::string>{"FollowPath"}},
       {"FollowPath.plugin",
        std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")},
       {"cmd_vel_topic", std::string("cmd_vel_nav")},
       {"controller_frequency", 50.0},
       {"general_goal_checker.xy_goal_tolerance", 0.20},
       {"general_goal_checker.yaw_goal_tolerance", 0.35}});
  krosc::nodes::nav2::BehaviorServer behavior(context);
  krosc::nodes::nav2::BtNavigator bt_navigator(
      context, "bt_navigator",
      {{"replanning_period", 1.0}, {"number_of_recoveries", 3}});
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
  krosc::nodes::CollisionMonitor collision(
      context, "collision_monitor",
      {{"cmd_vel_in_topic", std::string("cmd_vel_smoothed")},
       {"cmd_vel_out_topic", std::string("cmd_vel")},
       {"PolygonSlow.slowdown_ratio", 0.6},
       {"ignore_collision_during_teleop", true}});

  krosc::sensor::SensorManager sensors;
  sensors.add(imu, "sim_imu", 0.02);
  sensors.add(scan, "sim_scan", 0.05);
  krosc::control::ControlManager controls;
  controls.add(control, "sim_control", 0.02, 0.02);
  controls.set_control_hook(
      [&](double time, double period) { controller_manager.update(time, period); }, 0.01);

  simulator.start();
  sensors.start();
  if (!controls.start()) {
    std::cerr << "[krosc_ws_sim] control manager 시작 실패\n";
    sensors.stop();
    simulator.stop();
    return 1;
  }

  std::printf("[krosc_ws_sim] world: pucks=%d seed=%u port=%d\n", pucks, seed, port);
  std::printf("[krosc_ws_sim] 실행. Ctrl+C로 정지한다.\n");
  std::fflush(stdout);

  const auto start_time = std::chrono::steady_clock::now();
  while (!g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (duration > 0.0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() >=
            duration) {
      break;
    }
  }

  std::printf("[krosc_ws_sim] 정지 중...\n");
  std::fflush(stdout);
  controls.stop();
  sensors.stop();
  simulator.stop();
  context.shutdown();
  std::printf("[krosc_ws_sim] 종료\n");
  return 0;
}
