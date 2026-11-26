#include <QApplication>
#include <QTimer>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "krosc/context.hpp"
#include "krosc/node.hpp"
#include "krosc/simulator/simulator.hpp"
#include "krosc/simulator/snapshot.hpp"
#include "krosc/simulator/view_window.hpp"
#include "krosc/time/time.hpp"

#include "robot.hpp"
#include "sim_hardware.hpp"

#ifndef KROSC_WS_SIM_URDF
#define KROSC_WS_SIM_URDF "turtlebot3_waffle_pi.urdf"
#endif
#ifndef KROSC_WS_SIM_REAL_URDF
#define KROSC_WS_SIM_REAL_URDF "robot/description/urdf/turtlebot3_real.urdf"
#endif
#ifndef KROSC_WS_MAPS_DIR
#define KROSC_WS_MAPS_DIR "artifacts/maps"
#endif

namespace {

std::atomic<bool> g_stop{false};
void request_stop(int) { g_stop.store(true); }

}  // namespace

int main(int argc, char** argv) {
  QApplication application(argc, argv);

  int port = 9000;
  double duration = 0.0;
  int pucks = 10;
  std::uint32_t seed = 42;
  bool noisy = true;
  bool slam = false;
  bool arm = false;

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
    } else if (arg == "--slam") {
      slam = true;
    } else if (arg == "--arm") {
      arm = true;
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "usage: krosc_ws_simulator [--port 9000] [--duration SEC] [--pucks N]\n"
                   "                          [--seed N] [--no-noise] [--slam] [--arm]\n";
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
  std::unique_ptr<krosc::Context> context_holder;
  try {
    context_holder = std::make_unique<krosc::Context>(context_options);
  } catch (const std::exception& error) {
    std::cerr << "[krosc_ws_sim] 시작 실패: " << error.what() << "\n";
    return 1;
  }
  krosc::Context& context = *context_holder;
  krosc::Node ui_node(context, "sim_ui");

  krosc::simulator::SimulatorOptions sim_options;
  sim_options.noisy = noisy;
  sim_options.pucks = pucks;
  sim_options.seed = seed;
  sim_options.step_hz = 100.0;
  sim_options.urdf_path = KROSC_WS_SIM_URDF;
  sim_options.bridge = false;
  sim_options.odometry = false;
  sim_options.ekf = false;
  sim_options.publish_map = !slam;
  sim_options.publish_robot_state = !arm;
  krosc::simulator::Simulator simulator(context, sim_options);

  krosc_ws::sim::SimOpenCr opencr(simulator.robot());
  krosc_ws::sim::SimLds lds(simulator.robot());

  krosc_ws::RobotOptions options;
  options.urdf_path = KROSC_WS_SIM_URDF;
  options.maps_dir = KROSC_WS_MAPS_DIR;
  options.enable_moveit = arm;
  options.enable_map = slam;
  options.slam = slam;
  options.enable_rl = false;
  options.enable_arm = arm;
  options.enable_status = true;
  options.enable_robot_state_publisher = arm;
  if (arm) {
    options.urdf_path = KROSC_WS_SIM_REAL_URDF;
  }
  options.wheel_radius = krosc::simulator::TurtleBot3::kWheelRadius;
  options.wheel_separation = krosc::simulator::TurtleBot3::kWheelSeparation;
  options.scan_topic = "scan";
  options.scan_frame = "base_scan";
  options.scan_period = 0.1;
  options.imu_topic = "imu";
  options.imu_frame = "imu_link";
  options.imu_period = 0.02;
  options.control_read_period = 0.02;
  options.control_write_period = 0.02;
  options.control_period = 0.01;

  krosc_ws::Robot robot(context, opencr, &lds, options);
  krosc::simulator::SimWindow window(simulator.maze_world().walls(), simulator.robot(),
                                     simulator.snapshots(), ui_node);
  window.show();
  window.raise();
  window.activateWindow();
  window.view().setFocus();
  simulator.start();
  if (!robot.start()) {
    std::cerr << "[krosc_ws_sim] 로봇 시작 실패\n";
    simulator.stop();
    return 1;
  }

  std::printf("[krosc_ws_sim] world: pucks=%d seed=%u port=%d\n", pucks, seed, port);
  std::printf("[krosc_ws_sim] 실행. Ctrl+C로 정지한다.\n");
  std::fflush(stdout);

  const auto start_time = std::chrono::steady_clock::now();
  auto last_sim_status = std::chrono::steady_clock::now();
  while (!g_stop.load()) {
    application.processEvents();
    robot.log_status();
    const auto now = std::chrono::steady_clock::now();
    if (now - last_sim_status >= std::chrono::seconds(2)) {
      last_sim_status = now;
      if (auto snapshot = simulator.snapshots().latest()) {
        std::printf("[krosc_ws_sim] pose=%.2f,%.2f,%.1f° cmd=%.2f,%.2f\n",
                    snapshot->robot_pose.x, snapshot->robot_pose.y,
                    snapshot->robot_pose.theta * 180.0 / 3.141592653589793,
                    snapshot->command.linear.x, snapshot->command.angular.z);
        std::fflush(stdout);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (duration > 0.0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() >=
            duration) {
      break;
    }
  }

  std::printf("[krosc_ws_sim] 정지 중...\n");
  std::fflush(stdout);
  robot.stop();
  simulator.stop();
  window.close();
  context.shutdown();
  std::printf("[krosc_ws_sim] 종료\n");
  return 0;
}
