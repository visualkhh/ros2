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
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "srf/context.hpp"
#include "srf/msgs.hpp"
#include "srf/node.hpp"
#include "srf/qos.hpp"
#include "srf/tf/math.hpp"
#include "srf/simulator/runner.hpp"
#include "srf/simulator/simulator.hpp"
#include "srf/simulator/snapshot.hpp"
#include "srf/simulator/simulator_window.hpp"
#include "srf/time/time.hpp"

#include "panels/arm_panel.hpp"
#include "panels/camera_panel.hpp"
#include "engine.hpp"
#include "robot/waffle_pi_arm.hpp"
#include "hardwares/lds03/simulator_lds.hpp"
#include "hardwares/opencr/simulator_opencr.hpp"
#include "hardwares/realsense/simulator_realsense.hpp"
#include "world/recycling_world.hpp"

#ifndef SRF_RECYCLING_WS_URDF_PATH
#define SRF_RECYCLING_WS_URDF_PATH "engine/description/turtlebot3_manipulation_description/urdf/turtlebot3_manipulation.urdf"
#endif
#ifndef SRF_RECYCLING_WS_MAPS_DIR
#define SRF_RECYCLING_WS_MAPS_DIR "artifacts/maps"
#endif

namespace {

std::atomic<bool> g_stop{false};
void request_stop(int) { g_stop.store(true); }

// 재활용 월드 진실값. 시뮬레이션 스레드(listener)가 0.5초마다 만들고 메인 스레드는 복사만 한다.
struct TruthCache {
  std::mutex mutex;
  std::string json;
  srf::msgs::Path items;
  double next_time = 0.0;
};

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
    const srf::simulator::World& world,
    const srf_recycling_ws::sim::WafflePiArm& robot) {
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
  srf_recycling_ws::sim::SceneConfig scene;
  std::uint32_t camera_width = 424;
  std::uint32_t camera_height = 240;
  std::uint32_t seed = 42;
  bool noisy = true;
  bool slam = false;
  bool arm = true;  // srf_recycling_ws 로봇은 매니퓰레이터/MoveIt 포함이 기본

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) {
      port = std::stoi(argv[++i]);
    } else if (arg == "--duration" && i + 1 < argc) {
      duration = std::stod(argv[++i]);
    } else if (arg == "--scene" && i + 1 < argc) {
      const auto parsed = srf_recycling_ws::sim::parse_scene(argv[++i]);
      if (!parsed) {
        std::cerr << "--scene은 normal|multi|ambiguous|absent\n";
        return 2;
      }
      scene.scene = *parsed;
    } else if (arg == "--target" && i + 1 < argc) {
      scene.target = srf_recycling_ws::sim::parse_category(argv[++i]);
    } else if (arg == "--items" && i + 1 < argc) {
      scene.items = std::stoi(argv[++i]);
    } else if (arg == "--camera" && i + 1 < argc) {
      const std::string size = argv[++i];
      camera_width = static_cast<std::uint32_t>(std::stoul(size.substr(0, size.find('x'))));
      camera_height = static_cast<std::uint32_t>(std::stoul(size.substr(size.find('x') + 1)));
    } else if (arg == "--seed" && i + 1 < argc) {
      seed = static_cast<std::uint32_t>(std::stoul(argv[++i]));
    } else if (arg == "--no-noise") {
      noisy = false;
    } else if (arg == "--slam") {
      slam = true;
    } else if (arg == "--arm") {
      arm = true;
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "usage: srf_recycling_ws_simulator [--port 9000] [--duration SEC] [--seed N]\n"
                   "         [--scene normal|multi|ambiguous|absent] [--target can|pet|paper] [--items N]\n"
                   "         [--camera 424x240] [--no-noise] [--slam] [--arm]\n"
                   "  재배치: srf/recycling/reset 서비스 (String JSON {\"seed\":7,\"scene\":\"multi\",\"target\":\"can\",\"items\":5})\n"
                   "  진실값: recycling/truth 토픽 (String JSON, 2Hz)\n";
      return 0;
    }
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  srf::Context::Options context_options;
  context_options.name = "srf_recycling_ws_sim";
  context_options.executor_threads = 10;
  context_options.time_source = std::make_shared<srf::SimulatedTimeSource>();
  if (port > 0) {
    context_options.network.name = "srf_recycling_ws_sim";
    context_options.network.host = "0.0.0.0";
    context_options.network.port = static_cast<std::uint16_t>(port);
    context_options.network.receive_remap["cmd_vel"] = "cmd_vel_teleop";
  }
  std::unique_ptr<srf::Context> context_holder;
  try {
    context_holder = std::make_unique<srf::Context>(context_options);
  } catch (const std::exception& error) {
    std::cerr << "[srf_recycling_ws_sim] 시작 실패: " << error.what() << "\n";
    return 1;
  }
  srf::Context& context = *context_holder;
  srf::Node ui_node(context, "sim_ui");

  scene.seed = seed;
  auto world = std::make_unique<srf_recycling_ws::sim::RecyclingWorld>();
  world->randomize(scene);
  auto robot_holder = std::make_unique<srf_recycling_ws::sim::WafflePiArm>(noisy, seed, SRF_RECYCLING_WS_URDF_PATH);
  auto* robot_model = static_cast<srf_recycling_ws::sim::WafflePiArm*>(
      world->add(std::move(robot_holder), world->layout().robot_start));
  // MoveIt이 기본 제어자다. 패널에서 슬라이더를 움직이면 수동으로 전환된다.
  robot_model->set_manual_arm(!arm);

  srf::simulator::Simulator simulator(context, *world, *robot_model);

  srf_recycling_ws::sim::SimulatorOpenCr opencr(*robot_model, noisy, seed);
  srf_recycling_ws::sim::SimulatorLds lds(*robot_model, noisy, seed);
  srf_recycling_ws::sim::SimulatorRealSense camera(*robot_model, SRF_RECYCLING_WS_URDF_PATH, noisy, seed,
                                                   camera_width, camera_height);

  srf_recycling_ws::EngineOptions options;
  options.urdf_path = SRF_RECYCLING_WS_URDF_PATH;
  options.maps_dir = SRF_RECYCLING_WS_MAPS_DIR;
  options.enable_moveit = true;
  // 맵 목록/로드/저장은 SLAM 여부와 무관하게 항상 쓸 수 있어야 한다.
  options.enable_map = true;
  options.slam = slam;
  options.enable_rl = true;
  options.enable_arm = true;
  options.enable_status = true;
  options.enable_robot_state_publisher = true;
  options.enable_camera = true;
  options.wheel_radius = srf_recycling_ws::sim::WafflePiArm::kWheelRadius;
  options.wheel_separation = srf_recycling_ws::sim::WafflePiArm::kWheelSeparation;
  options.scan_topic = "scan";
  options.scan_frame = "base_scan";
  options.scan_period = 0.1;
  options.imu_topic = "imu";
  options.imu_frame = "imu_link";
  options.imu_period = 0.02;
  options.control_read_period = 0.02;
  options.control_write_period = 0.02;
  options.control_period = 0.01;

  srf_recycling_ws::Engine engine(context, opencr, &lds, options, &camera);
  // 상판 빈 자리는 시뮬레이션 스레드에서 매 스텝 계산해 두고, 서비스는 캐시만 읽는다.
  // (서비스 핸들러가 world.objects()를 직접 읽으면 시뮬레이션 스레드와 경쟁한다.)
  DeckFreeSpot deck_free;
  simulator.runner().add_listener(
      [&deck_free, robot_model](srf::simulator::World& current) {
        const auto spot = find_deck_free(current, *robot_model);
        if (spot) {
          deck_free.x.store(spot->first);
          deck_free.y.store(spot->second);
          deck_free.valid.store(true);
        } else {
          deck_free.valid.store(false);
        }
      });
  // 재활용 월드: 진실값(0.5초마다)과 재배치 서비스.
  TruthCache truth;
  simulator.runner().add_listener([&truth, &context](srf::simulator::World& current) {
    auto& recycling = static_cast<srf_recycling_ws::sim::RecyclingWorld&>(current);
    if (current.time() < truth.next_time) {
      return;
    }
    truth.next_time = current.time() + 0.5;
    srf::msgs::Path items;
    items.header = srf::msgs::Header{context.now(), "odom"};
    for (const auto* item : recycling.items()) {
      srf::msgs::PoseStamped pose;
      pose.header = items.header;
      pose.pose.position = {item->pose.x, item->pose.y, item->z + item->height / 2.0};
      pose.pose.orientation = srf::tfmath::from_yaw(item->pose.theta);
      items.poses.push_back(pose);
    }
    std::string json = recycling.truth_json(current.time());
    std::lock_guard<std::mutex> lock(truth.mutex);
    truth.json = std::move(json);
    truth.items = std::move(items);
  });
  ui_node.create_service<srf::msgs::String, srf::msgs::String>(
      "recycling/reset", [&simulator, &truth](const srf::msgs::String& request) {
        // {"seed":7,"scene":"multi","target":"can","items":5} — 빠진 값은 지금 값을 쓴다.
        const auto field = [&](const std::string& key) -> std::string {
          const auto at = request.data.find("\"" + key + "\"");
          if (at == std::string::npos) return {};
          auto begin = request.data.find(':', at) + 1;
          while (begin < request.data.size() && (request.data[begin] == ' ' || request.data[begin] == '"')) ++begin;
          auto end = begin;
          while (end < request.data.size() && request.data[end] != ',' && request.data[end] != '}' && request.data[end] != '"') ++end;
          return request.data.substr(begin, end - begin);
        };
        simulator.runner().post([field, &truth](srf::simulator::World& current) {
          auto& recycling = static_cast<srf_recycling_ws::sim::RecyclingWorld&>(current);
          auto config = recycling.scene();
          if (const auto v = field("seed"); !v.empty()) config.seed = static_cast<std::uint32_t>(std::stoul(v));
          if (const auto v = field("scene"); !v.empty()) config.scene = srf_recycling_ws::sim::parse_scene(v).value_or(config.scene);
          if (const auto v = field("target"); !v.empty()) config.target = srf_recycling_ws::sim::parse_category(v);
          if (const auto v = field("items"); !v.empty()) config.items = std::stoi(v);
          recycling.randomize(config);
          truth.next_time = 0.0;  // 다음 스텝에 진실값을 다시 만든다
        });
        return srf::msgs::String{R"({"ok":true})"};
      });
  ui_node.create_service<srf::msgs::String, srf::msgs::String>(
      "sim/deck_free", [&deck_free](const srf::msgs::String&) {
        if (!deck_free.valid.load()) {
          return srf::msgs::String{
              R"({"ok":false,"error":"상판에 빈 자리가 없다"})"};
        }
        char buffer[128];
        std::snprintf(buffer, sizeof(buffer),
                      "{\"ok\":true,\"x\":%.4f,\"y\":%.4f,\"yaw\":0}",
                      deck_free.x.load(), deck_free.y.load());
        return srf::msgs::String{buffer};
      });
  auto* arm_panel = new srf_recycling_ws::sim::ArmPanel(*robot_model);
  auto* camera_panel = new srf_recycling_ws::sim::CameraPanel(camera);
  srf::simulator::SimulatorWindow window(world->walls(), simulator.snapshots(), ui_node,
                                     "cmd_vel_teleop", {camera_panel, arm_panel});
  // Alt+드래그: 화면에서 퍽을 집어 바닥으로 옮긴다. 월드는 시뮬레이션 스레드만 만진다.
  window.view().set_move_object_handler([&simulator](int id, double x, double y, double z) {
    simulator.runner().post([id, x, y, z](srf::simulator::World& current) {
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
    std::cerr << "[srf_recycling_ws_sim] 로봇 시작 실패\n";
    simulator.stop();
    return 1;
  }

  std::printf("[srf_recycling_ws_sim] noise: body[%s] opencr[%s] lds[%s] camera[%s]\n",
              robot_model->noise_summary().c_str(), opencr.noise_summary().c_str(),
              lds.noise_summary().c_str(), camera.noise_summary().c_str());
  std::printf("[srf_recycling_ws_sim] world: recycling scene=%s target=%s items=%d seed=%u camera=%ux%u port=%d\n",
              srf_recycling_ws::sim::scene_name(scene.scene),
              scene.target ? srf_recycling_ws::sim::category_name(*scene.target) : "all", scene.items, seed,
              camera_width, camera_height, port);
  std::printf("[srf_recycling_ws_sim] 실행. Ctrl+C로 정지한다.\n");
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
      srf::msgs::Path path;
      std::string truth_json;
      {
        std::lock_guard<std::mutex> lock(truth.mutex);
        path = truth.items;
        truth_json = truth.json;
      }
      if (!truth_json.empty()) {
        ui_node.publish<srf::msgs::String>("recycling/truth", srf::msgs::String{std::move(truth_json)});
      }
      ui_node.publish<srf::msgs::Path>("sim_graspable", std::move(path));
    }
    if (now - last_sim_status >= std::chrono::seconds(2)) {
      last_sim_status = now;
      if (auto snapshot = simulator.snapshots().latest()) {
        std::printf("[srf_recycling_ws_sim] pose=%.2f,%.2f,%.1f° cmd=%.2f,%.2f\n",
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

  std::printf("[srf_recycling_ws_sim] 정지 중...\n");
  std::fflush(stdout);
  engine.stop();
  simulator.stop();
  window.close();
  context.shutdown();
  std::printf("[srf_recycling_ws_sim] 종료\n");
  return 0;
}
