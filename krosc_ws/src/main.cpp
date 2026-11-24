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
#include "krosc/net.hpp"
#include "krosc/nodes/cartographer.hpp"
#include "krosc/nodes/collision_monitor.hpp"
#include "krosc/nodes/map_catalog.hpp"
#include "krosc/nodes/moveit_lite.hpp"
#include "krosc/nodes/velocity_smoother.hpp"
#include "krosc/nodes/navigator.hpp"
#include "krosc/nodes/twist_mux.hpp"
#include "krosc/nodes/diff_drive_odometry.hpp"
#include "krosc/nodes/ekf.hpp"
#include "krosc/nodes/robot_state_publisher.hpp"
#include "krosc/time.hpp"

#include "robot/real_turtlebot3.hpp"
#include "robot_bridge.hpp"

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
  krosc::Context context(options);

  std::unique_ptr<krosc::net::Server> server;
  if (server_port > 0) {
    try {
      server = std::make_unique<krosc::net::Server>(context, static_cast<std::uint16_t>(server_port));
      std::printf("[krosc_ws] host: 0.0.0.0:%d\n", static_cast<int>(server->port()));
      const auto addresses = local_addresses();
      if (addresses.empty()) {
        std::printf("[krosc_ws] connect: <pi-ip>:%d\n", static_cast<int>(server->port()));
      } else {
        for (const auto& address : addresses) {
          std::printf("[krosc_ws] connect: %s:%d\n", address.c_str(), static_cast<int>(server->port()));
        }
      }
      std::printf("[krosc_ws] Mac: ./build/viz/krosc_viz_app --server <ip>:%d --fixed-frame odom\n",
                  static_cast<int>(server->port()));
      std::fflush(stdout);
    } catch (const std::exception& error) {
      std::fprintf(stderr, "[krosc_ws] network server를 열지 못했다: %s\n", error.what());
    }
  }

  krosc_ws::RealTurtleBot3 robot(opencr_port, lidar_port, enable_lidar);
  krosc_ws::RobotBridge bridge(context, robot);
  krosc::nodes::RobotStatePublisher robot_state(context, "RobotStatePublisher",
                                                {{"robot_description", urdf}});
  krosc::nodes::MoveItLite moveit(
      context, "moveit_lite",
      {{"base_link", std::string("base_link")},
       {"tip_link", std::string("end_effector_link")},
       {"joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4"}},
       {"lower_limits", std::vector<double>{-3.141592653589793, -2.20, -2.00, -2.00}},
       {"upper_limits", std::vector<double>{3.141592653589793, 2.20, 2.00, 2.20}},
       {"max_joint_speed", 0.35},
       {"max_joint_accel", 1.0},
       {"cartesian_speed", 0.08},
       {"cartesian_accel", 0.25},
       {"sample_time", 0.05}});
  krosc::nodes::DiffDriveOdometry odometry(
      context, "DiffDriveOdometry",
      {{"wheel_radius", krosc_ws::RealTurtleBot3::kWheelRadius},
       {"wheel_separation", krosc_ws::RealTurtleBot3::kWheelSeparation},
       {"publish_tf", false},
       {"twist_covariance_diagonal", std::vector<double>{0.005, 0.005, 1e6, 1e6, 1e6, 0.02}}});
  // IMU yaw rate는 정지 중에도 bias/진동이 커서 EKF에서 제외한다. 휠 오도메트리의
  // yaw 자세(5)를 절대 기준으로 쓰고, vx/vyaw(6/11)만 융합한다.
  krosc::nodes::Ekf ekf(context, "Ekf",
                        {{"odom0", std::string("odom")},
                         {"odom0_config", ekf_config({5, 6, 11})},
                         {"frequency", 20.0},
                         {"publish_tf", true}});
  constexpr double kDegreesToRadians = std::numbers::pi / 180.0;
  krosc::nodes::Cartographer cartographer(
      context, "Cartographer",
      {{"mode", std::string(slam ? "mapping" : "idle")},
       {"match_every", std::int64_t{4}},
       {"num_matching_points", std::int64_t{20}},
       {"linear_search_window", 0.05},
       {"angular_search_window", 3.0 * kDegreesToRadians},
       {"map_publish_period_sec", 2.0},
       {"tf_publish_period_sec", 0.05},
       {"tf_smoothing", 0.35}});
  krosc::nodes::MapCatalog maps(context, "map_catalog",
                                {{"maps_directory", maps_dir},
                                 {"map_frame", std::string("map")},
                                 {"list_period", 1.0}});
  krosc::nodes::Navigator navigator(context, "navigator",
                                    {{"map_topic", std::string("map")},
                                     {"cmd_vel_topic", std::string("cmd_vel_nav")},
                                     {"vx_max", 0.20},
                                     {"wz_max", 1.0},
                                     {"lookahead", 0.35},
                                     {"inflation", 0.24},
                                     {"xy_tolerance", 0.20},
                                     {"yaw_tolerance", 0.35}});
  krosc::nodes::TwistMux mux(context, "twist_mux", {{"cmd_vel_out", std::string("cmd_vel_mux")}});
  krosc::nodes::VelocitySmoother smoother(
      context, "velocity_smoother",
      {{"cmd_vel_in_topic", std::string("cmd_vel_mux")},
       {"cmd_vel_out_topic", std::string("cmd_vel_smoothed")},
       {"max_linear_accel", 0.5},
       {"max_angular_accel", 2.0},
       {"max_linear_speed", 0.26},
       {"max_angular_speed", 1.82},
       {"frequency", 50.0},
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
  constexpr auto kPeriod = std::chrono::milliseconds(20);
  auto next = std::chrono::steady_clock::now();
  auto last_status = std::chrono::steady_clock::now();
  while (!g_stop.load()) {
    robot.tick();
    bridge.poll();
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
  if (server) {
    server->shutdown();
  }
  context.shutdown();
  std::printf("[krosc_ws] 종료\n");
  return 0;
}
