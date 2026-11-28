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
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "srf/context.hpp"
#include "srf/msgs.hpp"
#include "srf/msgs/nav2_srv.hpp"
#include "srf/node.hpp"
#include "srf/qos.hpp"
#include "srf/tf/math.hpp"
#include "srf/simulator/runner.hpp"
#include "srf/simulator/simulator.hpp"
#include "srf/simulator/snapshot.hpp"
#include "srf/time/time.hpp"

#include "panels/arm_panel.hpp"
#include "panels/camera_panel.hpp"
#include "engine.hpp"
#include "robot/waffle_pi_arm.hpp"
#include "hardwares/lds03/simulator_lds.hpp"
#include "hardwares/opencr/simulator_opencr.hpp"
#include "hardwares/realsense/simulator_realsense.hpp"
#include "world/recycling_world.hpp"
#include "view/recycling_view.hpp"
#include "panels/cell_panel.hpp"

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
  // --headless: 창 없이 돈다 (서버·CI·데이터 생성). 시뮬레이션·카메라·네트워크·토픽은 그대로라 viz web으로 본다.
  bool headless = false;
  for (int i = 1; i < argc; ++i) {
    headless = headless || std::string(argv[i]) == "--headless";
  }
  if (headless) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
  }
  QApplication application(argc, argv);

  int port = 9000;
  double duration = 0.0;
  srf_recycling_ws::sim::SceneConfig scene;
  std::uint32_t camera_width = 424;
  std::uint32_t camera_height = 240;
  std::uint32_t seed = 42;
  std::string screenshot;  // 6초 뒤 창을 저장하고 끝낸다 (문서·확인용)
  std::string map_name = "recycling_facility";  // 시작할 때 불러올 지도 (빈 문자열이면 안 불러옴)
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
    } else if (arg == "--distractors" && i + 1 < argc) {
      scene.distractors = std::stoi(argv[++i]);
    } else if (arg == "--people" && i + 1 < argc) {
      scene.people = std::stoi(argv[++i]);
    } else if (arg == "--forklifts" && i + 1 < argc) {
      scene.forklifts = std::stoi(argv[++i]);
    } else if (arg == "--no-auto-supply") {
      scene.auto_supply = false;
    } else if (arg == "--no-auto-empty") {
      scene.auto_empty = false;
    } else if (arg == "--bin-capacity" && i + 1 < argc) {
      scene.bin_capacity = std::stoi(argv[++i]);
    } else if (arg == "--supply-items" && i + 1 < argc) {
      scene.supply_items = std::stoi(argv[++i]);
    } else if (arg == "--camera" && i + 1 < argc) {
      const std::string size = argv[++i];
      camera_width = static_cast<std::uint32_t>(std::stoul(size.substr(0, size.find('x'))));
      camera_height = static_cast<std::uint32_t>(std::stoul(size.substr(size.find('x') + 1)));
    } else if (arg == "--map" && i + 1 < argc) {
      map_name = argv[++i];
    } else if (arg == "--no-map") {
      map_name.clear();
    } else if (arg == "--screenshot" && i + 1 < argc) {
      screenshot = argv[++i];
    } else if (arg == "--seed" && i + 1 < argc) {
      seed = static_cast<std::uint32_t>(std::stoul(argv[++i]));
    } else if (arg == "--no-noise") {
      noisy = false;
    } else if (arg == "--slam") {
      slam = true;
    } else if (arg == "--arm") {
      arm = true;
    } else if (arg == "--headless") {
      // 위에서 이미 처리
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "usage: srf_recycling_ws_simulator [--port 9000] [--duration SEC] [--seed N]\n"
                   "         [--scene normal|multi|ambiguous|absent] [--target can|pet|paper] [--items N]\n"
                   "         [--distractors N] [--people 0-4] [--forklifts 0-1]\n"
                   "         [--no-auto-supply] [--no-auto-empty] [--bin-capacity 5] [--supply-items 4]\n"
                   "  협동 작업: recycling/request 서비스 (String JSON {\"action\":\"supply\"} | {\"action\":\"empty\",\"bin\":\"can\"})\n"
                   "         [--camera 424x240] [--no-noise] [--slam] [--arm] [--screenshot PATH] [--headless]\n"
                   "         [--map recycling_facility | --no-map]   (지도 + 지도 마커·구역을 불러오고 시작 위치를 알려 준다)\n"
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
  auto robot_holder = std::make_unique<srf_recycling_ws::sim::WafflePiArm>(noisy, seed, SRF_RECYCLING_WS_URDF_PATH,
                                                                           SRF_RECYCLING_WS_SRDF_PATH);
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
  // 시뮬 카메라는 color·depth 를 같은 렌더(수평 69°)로 만든다 → vlm/select 의 픽셀→3D 화각
  options.camera_color_hfov_deg = 69.0;
  options.camera_depth_hfov_deg = 69.0;
  options.camera_depth_vfov_deg = 2.0 * std::atan(std::tan(34.5 * 3.141592653589793 / 180.0) * camera_height / camera_width) *
                                  180.0 / 3.141592653589793;
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
  // 협동 작업: 스텝마다 셀 담당 작업자 동작·자동 요청 처리, 패널용 상태 복사 (0.2초마다).
  srf_recycling_ws::sim::SharedCell shared_cell;
  double cell_next = 0.0;
  simulator.runner().add_listener([&shared_cell, &cell_next](srf::simulator::World& current) {
    auto& recycling = static_cast<srf_recycling_ws::sim::RecyclingWorld&>(current);
    recycling.update();
    if (current.time() < cell_next) {
      return;
    }
    cell_next = current.time() + 0.2;
    srf_recycling_ws::sim::CellSnapshot snapshot;
    snapshot.bins = recycling.bin_status();
    snapshot.state = recycling.cell_state();
    snapshot.supplies = recycling.supplies();
    snapshot.collected = recycling.collected();
    snapshot.auto_supply = recycling.scene().auto_supply;
    snapshot.auto_empty = recycling.scene().auto_empty;
    std::lock_guard<std::mutex> lock(shared_cell.mutex);
    shared_cell.snapshot = std::move(snapshot);
  });
  ui_node.create_service<srf::msgs::String, srf::msgs::String>(
      "recycling/request", [&simulator](const srf::msgs::String& request) {
        // {"action":"supply"} | {"action":"empty","bin":"can|pet|paper"} — 셀 담당 작업자에게 일을 맡긴다
        const auto& text = request.data;
        const bool supply = text.find("\"supply\"") != std::string::npos;
        std::optional<srf_recycling_ws::sim::Category> bin;
        for (const char* name : {"can", "pet", "paper"}) {
          if (text.find(std::string("\"bin\":\"") + name + "\"") != std::string::npos ||
              text.find(std::string("\"bin\": \"") + name + "\"") != std::string::npos) {
            bin = srf_recycling_ws::sim::parse_category(name);
          }
        }
        if (!supply && !bin) {
          return srf::msgs::String{R"json({"ok":false,"error":"action은 supply 또는 empty(+bin: can|pet|paper)"})json"};
        }
        simulator.runner().post([supply, bin](srf::simulator::World& current) {
          auto& recycling = static_cast<srf_recycling_ws::sim::RecyclingWorld&>(current);
          supply ? recycling.request_supply() : recycling.request_empty(*bin);
        });
        return srf::msgs::String{R"({"ok":true})"};
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
          if (const auto v = field("distractors"); !v.empty()) config.distractors = std::stoi(v);
          if (const auto v = field("people"); !v.empty()) config.people = std::stoi(v);
          if (const auto v = field("forklifts"); !v.empty()) config.forklifts = std::stoi(v);
          if (const auto v = field("bin_capacity"); !v.empty()) config.bin_capacity = std::stoi(v);
          if (const auto v = field("supply_items"); !v.empty()) config.supply_items = std::stoi(v);
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
  // 진실 화면: 카메라와 같은 렌더러로 같은 장면을 그린다. 장면은 시뮬레이션 스레드가 30Hz로 복사해 둔다.
  srf_recycling_ws::sim::SharedScene shared_scene;
  simulator.runner().add_listener([&shared_scene, &camera, robot_model](srf::simulator::World& current) {
    if (current.time() < shared_scene.next_time) {
      return;
    }
    shared_scene.next_time = current.time() + 1.0 / 30.0;
    std::vector<srf_recycling_ws::sim::render::Instance> instances;
    static_cast<srf_recycling_ws::sim::RecyclingWorld&>(current).add_render_instances(instances);
    camera.robot_mesh().add_instances(robot_model->pose, robot_model->joint_positions(), instances);
    std::lock_guard<std::mutex> lock(shared_scene.mutex);
    shared_scene.instances = std::move(instances);
  });
  auto* cell_panel = new srf_recycling_ws::sim::CellPanel(
      [&simulator](std::function<void(srf_recycling_ws::sim::RecyclingWorld&)> work) {
        simulator.runner().post([work = std::move(work)](srf::simulator::World& current) {
          work(static_cast<srf_recycling_ws::sim::RecyclingWorld&>(current));
        });
      },
      shared_cell);
  srf_recycling_ws::sim::RecyclingWindow window(shared_scene, simulator.snapshots(), ui_node, "cmd_vel_teleop",
                                                {cell_panel, camera_panel, arm_panel});
  if (!headless) {
    window.show();
    window.raise();
    window.activateWindow();
    window.view().setFocus();
  }
  simulator.start();
  if (!engine.start()) {
    std::cerr << "[srf_recycling_ws_sim] 로봇 시작 실패\n";
    simulator.stop();
    return 1;
  }
  // 선별장 지도 + 지도 마커·구역 (map_catalog가 markers.yaml도 같이 불러온다). --slam이면 지도를 새로 만든다.
  if (!map_name.empty() && !slam) {
    try {
      const auto loaded = ui_node.call_service<srf::msgs::nav2::LoadMapRequest, srf::msgs::nav2::LoadMapResponse>(
          "map_catalog/load_map", srf::msgs::nav2::LoadMapRequest{map_name});
      if (loaded.result == srf::msgs::nav2::LoadMapResponse::RESULT_SUCCESS) {
        // 시작 위치를 알려 준다 (2D Pose Estimate와 같다). 막 시작해 odom TF가 아직 없으면 cartographer가
        // 버리므로 0.5초마다 몇 번 다시 보낸다 (같은 위치라 반복해도 무해).
        const auto& start = world->layout().robot_start;
        auto sends = std::make_shared<int>(0);
        auto timer = std::make_shared<srf::Timer>();
        *timer = ui_node.create_timer(0.5, [&ui_node, &context, start, sends, timer] {
          srf::msgs::PoseWithCovarianceStamped initial;
          initial.header = srf::msgs::Header{context.now(), "map"};
          initial.pose.pose.position = {start.x, start.y, 0.0};
          initial.pose.pose.orientation = srf::tfmath::from_yaw(start.theta);
          ui_node.publish("initialpose", initial);
          if (++*sends >= 6) {
            timer->cancel();
          }
        });
        std::printf("[srf_recycling_ws_sim] 지도 %s + 지도 마커 불러옴, 시작 위치 (%.2f, %.2f)\n", map_name.c_str(), start.x,
                    start.y);
      } else {
        std::printf("[srf_recycling_ws_sim] 지도 %s를 불러오지 못했다 (%d) — srf_recycling_ws_snapshot --export-map으로 만든다\n",
                    map_name.c_str(), loaded.result);
      }
    } catch (const std::exception& error) {
      std::printf("[srf_recycling_ws_sim] 지도 불러오기 실패: %s\n", error.what());
    }
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
    if (!screenshot.empty() &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() >= 6.0) {
      window.grab().save(QString::fromStdString(screenshot));
      std::printf("[srf_recycling_ws_sim] screenshot → %s\n", screenshot.c_str());
      break;
    }
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
