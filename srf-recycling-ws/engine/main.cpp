#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/types.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
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

#include "srf/bag/recorder.hpp"
#include "srf/context.hpp"
#ifdef SRF_RECYCLING_WS_WITH_ROS2
#include "srf/bridges/ros2/ros2_bridge.hpp"
#endif
#include "srf/msgs.hpp"
#include "srf/nodes/moveit/move_group.hpp"
#include "srf/qos.hpp"
#include "srf/nodes/cartographer.hpp"
#include "srf/nodes/collision_monitor.hpp"
#include "srf/nodes/ekf.hpp"
#include "srf/nodes/map_catalog.hpp"
#include "srf/nodes/map_saver.hpp"
#include "srf/nodes/map_server.hpp"
#include "srf/nodes/nav2/behavior_server.hpp"
#include "srf/nodes/nav2/bt_navigator.hpp"
#include "srf/nodes/nav2/controller_server.hpp"
#include "srf/nodes/nav2/costmap/costmap_2d_ros.hpp"
#include "srf/nodes/nav2/planner_server.hpp"
#include "srf/nodes/robot_state_publisher.hpp"
#include "srf/nodes/twist_mux.hpp"
#include "srf/nodes/velocity_smoother.hpp"
#include "srf/control/controller_manager.hpp"
#include "srf/time/time.hpp"

#include "engine.hpp"
#include "hardwares/lds03/lds03_device.hpp"
#include "hardwares/opencr/opencr_serial.hpp"
#ifdef SRF_RECYCLING_WS_HAS_REALSENSE
#include "hardwares/realsense/realsense_device.hpp"
#endif

#ifdef SRF_RECYCLING_WS_ENABLE_RL_NAV
#include "nodes/rl_nav/rl_nav_controller.hpp"
#endif

#ifndef SRF_RECYCLING_WS_URDF_PATH
#define SRF_RECYCLING_WS_URDF_PATH "engine/description/turtlebot3_manipulation_description/urdf/turtlebot3_manipulation.urdf"
#endif
#ifndef SRF_RECYCLING_WS_MAPS_DIR
#define SRF_RECYCLING_WS_MAPS_DIR "maps"
#endif

namespace {

std::atomic<bool> g_stop{false};

constexpr double kWheelRadius = 0.033;       // m
constexpr double kWheelSeparation = 0.287;   // m (실측)
constexpr double kMaxWheel = 7.8;            // rad/s
constexpr double kLidarMountX = 0.060;       // m, base_link 기준


void request_stop(int) { g_stop.store(true); }

std::string env_or(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  return value == nullptr ? fallback : std::string(value);
}

bool parse_count(const std::string& text, std::size_t& out) {
  try {
    const long value = std::stol(text);
    if (value < 0 || value > 64) {
      return false;
    }
    out = static_cast<std::size_t>(value);
    return true;
  } catch (...) {
    return false;
  }
}

std::size_t env_count(const char* name, std::size_t fallback) {
  const std::string text = env_or(name, "");
  std::size_t value = 0;
  return parse_count(text, value) ? value : fallback;
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
  std::cout << "usage: srf_recycling_ws_real [--opencr PORT] [--lidar PORT] [--port PORT] [--urdf PATH] [--maps DIR]\n"
               "                  [--scan-topic NAME] [--scan-rate HZ] [--imu-topic NAME] [--imu-rate HZ]\n"
               "                  [--camera-serial SERIAL] [--camera-rate HZ] [--camera-depth-rate HZ]\n"
               "                  [--camera-width PX] [--camera-height PX] [--camera-fps N]\n"
               "                  [--camera-quality N] [--camera-color-decimation N] [--camera-depth-decimation N]\n"
               "                  [--executor-threads N]\n"
               "                  [--no-lidar] [--no-camera] [--slam] [--ros2] [--ros2-namespace NS]\n"
               "  --slam: SLAM(mapping)을 켠다 (없으면 idle). 원격 viz는 --fixed-frame map\n"
               "  --ros2: ROS 2 브리지를 프로세스 안에서 켠다 (SRF_RECYCLING_WS_WITH_ROS2=ON 빌드만).\n"
               "          ROS /cmd_vel은 cmd_vel_teleop으로 들어가 twist_mux·collision_monitor를 거친다\n"
               "  --ros2-namespace: ROS 2 쪽 토픽 접두어 (예: /pak → /pak/scan)\n"
               "  --record PATH: 로봇 프로세스 안에서 모든 토픽을 MCAP 파일에 녹화 (srf bag info/play로 본다)\n"
               "  --token TOKEN: 같은 토큰을 가진 피어만 붙게 한다 (env SRF_RECYCLING_WS_TOKEN, 비우면 인증 없음)\n"
               "  --no-announce: UDP 9099로 위치를 알리지 않는다 (srf discover / --robot auto에 안 보임)\n"
               "  --camera-rate: color Image 발행 주기 (기본 5Hz)\n"
               "  --camera-depth-rate: depth Image 발행 주기 (기본 2Hz)\n"
               "  --camera-width/height: D435 stream 크기 (기본 640x480@15)\n"
               "  --camera-color-decimation: color 축소 배율 (기본 2 -> 320x240 JPEG)\n"
               "  --camera-depth-decimation: depth 축소 배율 (기본 4 -> 160x120)\n"
               "env: OPENCR_PORT (/dev/opencr), LIDAR_PORT (auto), SRF_RECYCLING_WS_SERVER_PORT (9000), SRF_RECYCLING_WS_MAPS_DIR,\n"
               "     SRF_RECYCLING_WS_SCAN_TOPIC (scan), SRF_RECYCLING_WS_IMU_TOPIC (imu), SRF_RECYCLING_WS_CAMERA (1), SRF_RECYCLING_WS_CAMERA_SERIAL,\n"
               "     SRF_RECYCLING_WS_CAMERA_RATE (5), SRF_RECYCLING_WS_CAMERA_DEPTH_RATE (2), SRF_RECYCLING_WS_EXECUTOR_THREADS (10),\n"
               "     SRF_RECYCLING_WS_GRASP_DETECTOR (1), SRF_RECYCLING_WS_OLLAMA_URL, SRF_RECYCLING_WS_OLLAMA_MODEL, SRF_RECYCLING_WS_LLM_MODE (llm|hybrid|rule),\n"
               "     SRF_RECYCLING_WS_VLA_URL (http://127.0.0.1:8765/plan), SRF_RECYCLING_WS_VLM_URL (http://127.0.0.1:8766/query)\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string opencr_port = env_or("OPENCR_PORT", "/dev/opencr");
  std::string lidar_port = env_or("LIDAR_PORT", "");
  std::string urdf_path = SRF_RECYCLING_WS_URDF_PATH;
  std::string maps_dir = env_or("SRF_RECYCLING_WS_MAPS_DIR", SRF_RECYCLING_WS_MAPS_DIR);
  int server_port = 9000;
  {
    const std::string port_env = env_or("SRF_RECYCLING_WS_SERVER_PORT", "9000");
    try {
      server_port = std::stoi(port_env);
    } catch (...) {
      server_port = 9000;
    }
  }
  bool enable_lidar = true;
  bool enable_camera = env_or("SRF_RECYCLING_WS_CAMERA", "1") != "0";
  bool slam = false;
  bool enable_ros2 = env_or("SRF_RECYCLING_WS_ROS2", "0") != "0";
  std::string record_path;
  std::string token = env_or("SRF_RECYCLING_WS_TOKEN", "");
  bool announce = env_or("SRF_RECYCLING_WS_ANNOUNCE", "1") != "0";
  std::string ros2_namespace = env_or("SRF_RECYCLING_WS_ROS2_NAMESPACE", "");
  std::string scan_topic = env_or("SRF_RECYCLING_WS_SCAN_TOPIC", "scan");
  std::string imu_topic = env_or("SRF_RECYCLING_WS_IMU_TOPIC", "imu");
  std::string camera_serial = env_or("SRF_RECYCLING_WS_CAMERA_SERIAL", "");
  double scan_rate = 10.0;
  double imu_rate = 50.0;
  double camera_rate = 5.0;
  double camera_depth_rate = 2.0;
  int camera_width = 640;
  int camera_height = 480;
  int camera_fps = 15;
  [[maybe_unused]] int camera_quality = 70;
  int camera_color_decimation = 2;
  int camera_depth_decimation = 4;
  try {
    camera_rate = std::stod(env_or("SRF_RECYCLING_WS_CAMERA_RATE", "5"));
    camera_depth_rate = std::stod(env_or("SRF_RECYCLING_WS_CAMERA_DEPTH_RATE", "2"));
  } catch (...) {
    camera_rate = 5.0;
    camera_depth_rate = 2.0;
  }
  std::size_t executor_threads = env_count("SRF_RECYCLING_WS_EXECUTOR_THREADS", 10);
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
    } else if (argument == "--scan-topic" && i + 1 < argc) {
      scan_topic = argv[++i];
    } else if (argument == "--scan-rate" && i + 1 < argc) {
      scan_rate = std::stod(argv[++i]);
    } else if (argument == "--imu-topic" && i + 1 < argc) {
      imu_topic = argv[++i];
    } else if (argument == "--imu-rate" && i + 1 < argc) {
      imu_rate = std::stod(argv[++i]);
    } else if (argument == "--camera-serial" && i + 1 < argc) {
      camera_serial = argv[++i];
    } else if (argument == "--camera-rate" && i + 1 < argc) {
      camera_rate = std::stod(argv[++i]);
    } else if (argument == "--camera-depth-rate" && i + 1 < argc) {
      camera_depth_rate = std::stod(argv[++i]);
    } else if (argument == "--camera-width" && i + 1 < argc) {
      camera_width = std::stoi(argv[++i]);
    } else if (argument == "--camera-height" && i + 1 < argc) {
      camera_height = std::stoi(argv[++i]);
    } else if (argument == "--camera-fps" && i + 1 < argc) {
      camera_fps = std::stoi(argv[++i]);
    } else if (argument == "--camera-quality" && i + 1 < argc) {
      camera_quality = std::stoi(argv[++i]);
    } else if (argument == "--camera-color-decimation" && i + 1 < argc) {
      camera_color_decimation = std::stoi(argv[++i]);
    } else if (argument == "--camera-depth-decimation" && i + 1 < argc) {
      camera_depth_decimation = std::stoi(argv[++i]);
    } else if (argument == "--executor-threads" && i + 1 < argc) {
      if (!parse_count(argv[++i], executor_threads)) {
        std::cerr << "[srf_recycling_ws] executor-threads는 0..64 범위여야 한다\n";
        return 2;
      }
    } else if (argument == "--no-lidar") {
      enable_lidar = false;
    } else if (argument == "--no-camera") {
      enable_camera = false;
    } else if (argument == "--no-announce") {
      announce = false;
    } else if (argument == "--token" && i + 1 < argc) {
      token = argv[++i];
    } else if (argument == "--record" && i + 1 < argc) {
      record_path = argv[++i];
    } else if (argument == "--ros2") {
      enable_ros2 = true;
    } else if (argument == "--ros2-namespace" && i + 1 < argc) {
      ros2_namespace = argv[++i];
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
  if (!(scan_rate > 0.0) || !(imu_rate > 0.0) ||
      (enable_camera &&
       (!(camera_rate > 0.0) || !(camera_depth_rate > 0.0) || camera_width <= 0 ||
        camera_height <= 0 || camera_fps <= 0 || camera_color_decimation <= 0 ||
        camera_depth_decimation <= 0))) {
    std::cerr << "[srf_recycling_ws] scan/imu/camera rate·해상도·decimation은 0보다 커야 한다\n";
    return 2;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  srf::Context::Options options;
  options.name = "srf_recycling_ws";
  options.executor_threads = executor_threads;
  options.time_source = std::make_shared<srf::SystemTimeSource>();
  if (server_port > 0) {
    options.network.name = "srf_recycling_ws";
    options.network.host = "0.0.0.0";
    options.network.port = static_cast<std::uint16_t>(server_port);
    // Viz의 텔레옵은 cmd_vel로 오지만 로봇에서는 cmd_vel_teleop으로 받아야 한다.
    options.network.receive_remap["cmd_vel"] = "cmd_vel_teleop";
    options.network.token = token;  // 비우면 인증 없음
    options.network.announce = announce;
  }
  srf::Context context(options);

  // bag 녹화: 노드(Engine)보다 먼저 시작해 생성 중 latched 토픽도 담는다.
  std::unique_ptr<srf::bag::Recorder> recorder;
  if (!record_path.empty()) {
    srf::bag::RecorderOptions record_options;
    record_options.path = record_path;
    recorder = std::make_unique<srf::bag::Recorder>(std::move(record_options));
    recorder->start(context);
  }

  // ROS 2 브리지: 노드(Engine)보다 먼저 시작해 생성 중 latched 토픽(map, tf_static)도 넘긴다.
#ifdef SRF_RECYCLING_WS_WITH_ROS2
  std::unique_ptr<srf::ros2::Ros2Bridge> ros2_bridge;
  if (enable_ros2) {
    srf::ros2::Ros2BridgeOptions bridge_options;
    bridge_options.ros_namespace = ros2_namespace;
    for (auto& route : bridge_options.topics) {
      if (route.topic == "cmd_vel") {
        // 이 로봇의 SRF cmd_vel은 collision_monitor를 거친 최종 출력이다. ROS 명령은 teleop 입력으로 받는다.
        route.ros_topic = ros2_namespace + "/cmd_vel";
        route.topic = "cmd_vel_teleop";
      }
    }
    ros2_bridge = std::make_unique<srf::ros2::Ros2Bridge>(std::move(bridge_options));
    ros2_bridge->start(context);
    std::printf("[srf_recycling_ws] ROS 2 bridge: on (namespace '%s')\n", ros2_namespace.c_str());
  }
#else
  if (enable_ros2) {
    std::cerr << "[srf_recycling_ws] --ros2: ROS 2 브리지 없이 빌드됨 (SRF_RECYCLING_WS_WITH_ROS2=ON으로 빌드)\n";
    return 2;
  }
#endif

  if (const auto* peer = context.peer_network()) {
    const int port = static_cast<int>(peer->port());
    std::printf("[srf_recycling_ws] peer listen: 0.0.0.0:%d\n", port);
    const auto addresses = local_addresses();
    if (addresses.empty()) {
      std::printf("[srf_recycling_ws] peer: <pi-ip>:%d\n", port);
    } else {
      for (const auto& address : addresses) {
        std::printf("[srf_recycling_ws] peer: %s:%d\n", address.c_str(), port);
      }
    }
    std::printf("[srf_recycling_ws] Viz: --server <ip>:%d\n", port);
    std::fflush(stdout);
  }

  srf_recycling_ws::OpenCrSerial opencr(kWheelRadius, kWheelSeparation, std::move(opencr_port), 115200,
                                0.02, 0.5, -1.0, -1.0, kMaxWheel);
  std::unique_ptr<srf_recycling_ws::Lds03Device> lidar;
  if (enable_lidar) {
    lidar = std::make_unique<srf_recycling_ws::Lds03Device>(std::move(lidar_port), kLidarMountX, true,
                                                    true);
  }
  srf_recycling_ws::RealSenseCamera* camera = nullptr;
#ifdef SRF_RECYCLING_WS_HAS_REALSENSE
  std::unique_ptr<srf_recycling_ws::RealSenseDevice> camera_owner;
  if (enable_camera) {
    srf_recycling_ws::RealSenseDevice::Options camera_options;
    camera_options.serial = camera_serial;
    camera_options.width = camera_width;
    camera_options.height = camera_height;
    camera_options.fps = camera_fps;
    camera_options.jpeg_quality = camera_quality;
    camera_options.color_decimation = camera_color_decimation;
    camera_options.depth_decimation = camera_depth_decimation;
    camera_owner = std::make_unique<srf_recycling_ws::RealSenseDevice>(std::move(camera_options));
    camera = camera_owner.get();
  }
#else
  if (enable_camera) {
    std::fprintf(stderr, "[srf_recycling_ws] RealSense 없이 빌드됨 — camera를 끈다 (librealsense2 필요)\n");
    enable_camera = false;
  }
#endif

  srf_recycling_ws::EngineOptions engine_options;
  engine_options.urdf_path = urdf_path;
  engine_options.maps_dir = maps_dir;
  engine_options.slam = slam;
  engine_options.enable_moveit = true;
  engine_options.enable_map = true;
  engine_options.enable_rl = true;
  engine_options.enable_arm = true;
  engine_options.enable_status = true;
  engine_options.scan_topic = scan_topic;
  engine_options.scan_frame = "base_scan";
  engine_options.scan_period = 1.0 / scan_rate;
  engine_options.imu_topic = imu_topic;
  engine_options.imu_frame = "imu_link";
  engine_options.imu_period = 1.0 / imu_rate;
  engine_options.enable_camera = enable_camera;
  engine_options.camera_period = 1.0 / camera_rate;
  engine_options.camera_depth_period = 1.0 / camera_depth_rate;
  // 실기 D435 인식: color/depth에서 물체를 찾아 pick에 넘긴다.
  // 카메라가 없거나 인식만 끄고 싶으면 SRF_RECYCLING_WS_GRASP_DETECTOR=0.
  engine_options.enable_grasp_detector = env_or("SRF_RECYCLING_WS_GRASP_DETECTOR", "1") != "0";
  // 사이드카/LLM은 Pi 밖(PC/GPU 서버)에서 돌릴 수 있다.
  engine_options.llm_ollama_url =
      env_or("SRF_RECYCLING_WS_OLLAMA_URL", engine_options.llm_ollama_url);
  engine_options.llm_model = env_or("SRF_RECYCLING_WS_OLLAMA_MODEL", engine_options.llm_model);
  engine_options.llm_mode = env_or("SRF_RECYCLING_WS_LLM_MODE", engine_options.llm_mode);
  engine_options.llm_include_image = env_or("SRF_RECYCLING_WS_LLM_IMAGE", "1") != "0";
  try {
    engine_options.llm_timeout = std::max(
        1.0, std::stod(env_or("SRF_RECYCLING_WS_LLM_TIMEOUT", std::to_string(engine_options.llm_timeout))));
  } catch (const std::exception&) {
    // 기본값 유지
  }
  engine_options.vla_url = env_or("SRF_RECYCLING_WS_VLA_URL", engine_options.vla_url);
  engine_options.vlm_url = env_or("SRF_RECYCLING_WS_VLM_URL", engine_options.vlm_url);
  try {
    engine_options.vla_replan_steps = static_cast<std::size_t>(
        std::max(0, std::stoi(env_or("SRF_RECYCLING_WS_VLA_REPLAN_STEPS",
                                     std::to_string(engine_options.vla_replan_steps)))));
  } catch (const std::exception&) {
    // 기본값 유지
  }
  engine_options.control_read_period = 0.02;
  engine_options.control_write_period = 0.02;
  engine_options.control_period = 0.01;

  srf_recycling_ws::Engine engine(context, opencr, lidar.get(), engine_options, camera);
  if (!engine.start()) {
    std::cerr << "[srf_recycling_ws] 엔진 시작 실패\n";
    return 1;
  }
  std::printf("[srf_recycling_ws] sensors: scan=%s %.1fHz, imu=%s %.1fHz\n", scan_topic.c_str(),
              scan_rate, imu_topic.c_str(), imu_rate);
  if (camera) {
    std::printf("[srf_recycling_ws] camera: color=%s %.1fHz %dx%d, depth=%s %.1fHz %dx%d\n",
                engine_options.camera_color_topic.c_str(), camera_rate,
                camera_width / std::max(1, camera_color_decimation),
                camera_height / std::max(1, camera_color_decimation),
                engine_options.camera_depth_topic.c_str(), camera_depth_rate,
                camera_width / std::max(1, camera_depth_decimation),
                camera_height / std::max(1, camera_depth_decimation));
  } else {
    std::printf("[srf_recycling_ws] camera: off\n");
  }
  std::printf("[srf_recycling_ws] executor threads: %zu\n", executor_threads);
  std::printf("[srf_recycling_ws] SLAM: %s\n", slam ? "mapping" : "idle");
  std::printf("[srf_recycling_ws] maps: %s\n", maps_dir.c_str());
  std::printf("[srf_recycling_ws] 실행. Ctrl+C로 정지한다.\n");
  std::fflush(stdout);
  constexpr auto kPeriod = std::chrono::milliseconds(10);
  auto next = std::chrono::steady_clock::now();
  while (!g_stop.load()) {
    engine.log_status();
    next += kPeriod;
    std::this_thread::sleep_until(next);
    const auto now = std::chrono::steady_clock::now();
    if (now - next > std::chrono::milliseconds(250)) {
      next = now;
    }
  }

  std::printf("[srf_recycling_ws] 정지 중...\n");
  std::fflush(stdout);
#ifdef SRF_RECYCLING_WS_WITH_ROS2
  if (ros2_bridge) {
    ros2_bridge->shutdown();  // 노드보다 먼저 멈춘다
  }
#endif
  engine.stop();
  if (recorder) {
    recorder->shutdown();  // 쌓인 것을 다 쓰고 파일을 닫는다
  }
  context.shutdown();
  std::printf("[srf_recycling_ws] 종료\n");
  return 0;
}
