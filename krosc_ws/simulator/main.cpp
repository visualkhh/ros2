#include <QApplication>
#include <QTimer>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "krosc/context.hpp"
#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf/math.hpp"
#include "krosc/simulator/worlds/maze_world.hpp"
#include "krosc/simulator/runner.hpp"
#include "krosc/simulator/simulator.hpp"
#include "krosc/simulator/snapshot.hpp"
#include "krosc/simulator/simulator_window.hpp"
#include "krosc/time/time.hpp"

#include "panels/arm_panel.hpp"
#include "panels/camera_panel.hpp"
#include "engine.hpp"
#include "robot/waffle_pi_arm.hpp"
#include "hardwares/lds03/simulator_lds.hpp"
#include "hardwares/opencr/simulator_opencr.hpp"
#include "hardwares/realsense/simulator_realsense.hpp"

#ifndef KROSC_WS_URDF_PATH
#define KROSC_WS_URDF_PATH "simulator/robot/urdf/turtlebot3_real.urdf"
#endif
#ifndef KROSC_WS_MAPS_DIR
#define KROSC_WS_MAPS_DIR "artifacts/maps"
#endif

namespace {

std::atomic<bool> g_stop{false};
void request_stop(int) { g_stop.store(true); }

// 상판 빈 자리. 시뮬레이션 스레드(listener)가 갱신하고 서비스는 이 값만 읽는다.
struct DeckFreeSpot {
  std::atomic<bool> valid{false};
  std::atomic<double> x{0.05};
  std::atomic<double> y{0.0};
};

// 고정물(LDS/팔 베이스)과 이미 올려둔 payload를 피해 상판 위 빈 자리를 고른다.
// D435는 상판보다 낮아서 상판 공간을 차지하지 않는다. 로봇은 상판을 볼 수 없으므로
// 카메라 대신 로봇 모델+payload로 계산한다.
std::optional<std::pair<double, double>> find_deck_free(
    const krosc::simulator::World& world,
    const krosc_ws::sim::WafflePiArm& robot) {
  constexpr double kPuckRadius = 0.02;
  constexpr double kMargin = 0.015;
  constexpr double kMinArmReach = 0.09;   // 팔 베이스에서 너무 가까우면 IK 불가
  constexpr double kMaxArmReach = 0.32;
  constexpr double kLdsRadius = 0.034;    // LDS 박스 반대각
  const double robot_x = robot.pose.x;
  const double robot_y = robot.pose.y;
  const double robot_yaw = robot.pose.theta;
  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);
  const std::array<double, 11> xs{0.05, 0.04, 0.03, 0.02, 0.01, 0.0,
                                  -0.01, -0.02, -0.03, -0.04, -0.05};
  const std::array<double, 7> ys{0.0, 0.04, -0.04, 0.08, -0.08, 0.10, -0.10};
  for (double bx : xs) {
    for (double by : ys) {
      const double arm_distance = std::hypot(bx + 0.092, by);
      if (arm_distance < kMinArmReach + kPuckRadius + kMargin ||
          arm_distance > kMaxArmReach) {
        continue;
      }
      if (std::hypot(bx + 0.024, by) < kLdsRadius + kPuckRadius + kMargin) {
        continue;
      }
      bool free = true;
      for (const auto& object : world.objects()) {
        if (!object || !object->on_robot) {
          continue;
        }
        const double dx = object->pose.x - robot_x;
        const double dy = object->pose.y - robot_y;
        const double local_x = c * dx + s * dy;
        const double local_y = -s * dx + c * dy;
        if (std::hypot(bx - local_x, by - local_y) <
            object->radius + kPuckRadius + kMargin) {
          free = false;
          break;
        }
      }
      if (free) {
        return std::make_pair(bx, by);
      }
    }
  }
  return std::nullopt;
}

}  // namespace

int main(int argc, char** argv) {
  QApplication application(argc, argv);

  int port = 9000;
  double duration = 0.0;
  int pucks = 10;
  std::uint32_t seed = 42;
  bool noisy = true;
  bool slam = false;
  bool arm = true;  // krosc_ws 로봇은 매니퓰레이터/MoveIt 포함이 기본

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

  auto world = std::make_unique<krosc::simulator::MazeWorld>(pucks, seed);
  auto robot_holder = std::make_unique<krosc_ws::sim::WafflePiArm>(noisy, seed, KROSC_WS_URDF_PATH);
  auto* robot_model = static_cast<krosc_ws::sim::WafflePiArm*>(
      world->add(std::move(robot_holder)));
  // MoveIt이 기본 제어자다. 패널에서 슬라이더를 움직이면 수동으로 전환된다.
  robot_model->set_manual_arm(!arm);

  krosc::simulator::Simulator simulator(context, *world, *robot_model);

  krosc_ws::sim::SimulatorOpenCr opencr(*robot_model, noisy, seed);
  krosc_ws::sim::SimulatorLds lds(*robot_model, noisy, seed);
  krosc_ws::sim::SimulatorRealSense camera(*robot_model, noisy, seed);

  krosc_ws::EngineOptions options;
  options.urdf_path = KROSC_WS_URDF_PATH;
  options.maps_dir = KROSC_WS_MAPS_DIR;
  options.enable_moveit = true;
  // 맵 목록/로드/저장은 SLAM 여부와 무관하게 항상 쓸 수 있어야 한다.
  options.enable_map = true;
  options.slam = slam;
  options.enable_rl = true;
  options.enable_arm = true;
  options.enable_status = true;
  options.enable_robot_state_publisher = true;
  options.enable_camera = true;
  options.wheel_radius = krosc_ws::sim::WafflePiArm::kWheelRadius;
  options.wheel_separation = krosc_ws::sim::WafflePiArm::kWheelSeparation;
  options.scan_topic = "scan";
  options.scan_frame = "base_scan";
  options.scan_period = 0.1;
  options.imu_topic = "imu";
  options.imu_frame = "imu_link";
  options.imu_period = 0.02;
  options.control_read_period = 0.02;
  options.control_write_period = 0.02;
  options.control_period = 0.01;

  krosc_ws::Engine engine(context, opencr, &lds, options, &camera);
  // 상판 빈 자리는 시뮬레이션 스레드에서 매 스텝 계산해 두고, 서비스는 캐시만 읽는다.
  // (서비스 핸들러가 world.objects()를 직접 읽으면 시뮬레이션 스레드와 경쟁한다.)
  DeckFreeSpot deck_free;
  simulator.runner().add_listener(
      [&deck_free, robot_model](krosc::simulator::World& current) {
        const auto spot = find_deck_free(current, *robot_model);
        if (spot) {
          deck_free.x.store(spot->first);
          deck_free.y.store(spot->second);
          deck_free.valid.store(true);
        } else {
          deck_free.valid.store(false);
        }
      });
  ui_node.create_service<krosc::msgs::String, krosc::msgs::String>(
      "sim/deck_free", [&deck_free](const krosc::msgs::String&) {
        if (!deck_free.valid.load()) {
          return krosc::msgs::String{
              R"({"ok":false,"error":"상판에 빈 자리가 없다"})"};
        }
        char buffer[128];
        std::snprintf(buffer, sizeof(buffer),
                      "{\"ok\":true,\"x\":%.4f,\"y\":%.4f,\"yaw\":0}",
                      deck_free.x.load(), deck_free.y.load());
        return krosc::msgs::String{buffer};
      });
  auto* arm_panel = new krosc_ws::sim::ArmPanel(*robot_model);
  auto* camera_panel = new krosc_ws::sim::CameraPanel(camera);
  krosc::simulator::SimulatorWindow window(world->walls(), simulator.snapshots(), ui_node,
                                     "cmd_vel_teleop", {camera_panel, arm_panel});
  // Alt+드래그: 화면에서 퍽을 집어 바닥으로 옮긴다. 월드는 시뮬레이션 스레드만 만진다.
  window.view().set_move_object_handler([&simulator](int id, double x, double y, double z) {
    simulator.runner().post([id, x, y, z](krosc::simulator::World& current) {
      auto& objects = current.objects();
      if (id < 0 || static_cast<std::size_t>(id) >= objects.size()) {
        return;
      }
      auto& object = objects[static_cast<std::size_t>(id)];
      if (!object || object->held || object->on_robot) {
        return;
      }
      object->pose.x = x;
      object->pose.y = y;
      object->z = std::clamp(z, 0.0, 1.0);
      object->velocity = {0.0, 0.0};
    });
  });
  window.show();
  window.raise();
  window.activateWindow();
  window.view().setFocus();
  simulator.start();
  if (!engine.start()) {
    std::cerr << "[krosc_ws_sim] 로봇 시작 실패\n";
    simulator.stop();
    return 1;
  }

  std::printf("[krosc_ws_sim] noise: body[%s] opencr[%s] lds[%s] camera[%s]\n",
              robot_model->noise_summary().c_str(), opencr.noise_summary().c_str(),
              lds.noise_summary().c_str(), camera.noise_summary().c_str());
  std::printf("[krosc_ws_sim] world: pucks=%d seed=%u port=%d\n", pucks, seed, port);
  std::printf("[krosc_ws_sim] 실행. Ctrl+C로 정지한다.\n");
  std::fflush(stdout);

  const auto start_time = std::chrono::steady_clock::now();
  auto last_sim_status = std::chrono::steady_clock::now();
  auto last_objects = last_sim_status;
  while (!g_stop.load()) {
    application.processEvents();
    engine.log_status();
    const auto now = std::chrono::steady_clock::now();
    if (now - last_objects >= std::chrono::milliseconds(250)) {
      last_objects = now;
      krosc::msgs::Path path;
      path.header = krosc::msgs::Header{context.now(), "odom"};
      if (const auto* object = world->graspable(); object != nullptr) {
        krosc::msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position = {object->pose.x, object->pose.y,
                              object->z + object->height / 2.0};
        pose.pose.orientation = krosc::tfmath::from_yaw(object->pose.theta);
        path.poses.push_back(pose);
      }
      ui_node.publish<krosc::msgs::Path>("sim_graspable", std::move(path));
    }
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
  engine.stop();
  simulator.stop();
  window.close();
  context.shutdown();
  std::printf("[krosc_ws_sim] 종료\n");
  return 0;
}
