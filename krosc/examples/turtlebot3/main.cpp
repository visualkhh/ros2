#include <QApplication>
#include <QTimer>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/nodes/diff_drive_odometry.hpp"
#include "krosc/nodes/ekf.hpp"
#include "krosc/nodes/robot_state_publisher.hpp"
#include "krosc/sim/maze_world.hpp"
#include "krosc/sim/runner.hpp"
#include "krosc/sim/snapshot.hpp"
#include "krosc/sim/turtlebot3.hpp"
#include "krosc/sim/view_window.hpp"
#include "krosc/time.hpp"
#include "krosc/viz/viz_window.hpp"

#include "map_publisher.hpp"
#include "robot_bridge.hpp"

#ifndef KROSC_TURTLEBOT3_URDF
#define KROSC_TURTLEBOT3_URDF "description/turtlebot3_waffle_pi.urdf"
#endif

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
  for (int index : enabled) {
    if (index >= 0 && index < 15) {
      config[static_cast<std::size_t>(index)] = true;
    }
  }
  return config;
}

}  // namespace

int main(int argc, char** argv) {
  QApplication application(argc, argv);
  bool headless = false;
  bool noisy = true;
  std::string screenshot;
  std::string screenshot_sim;
  double duration = 0.0;
  double zoom = 0.0;
  int server_port = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--headless") {
      headless = true;
    } else if (argument == "--no-noise") {
      noisy = false;
    } else if (argument == "--screenshot" && i + 1 < argc) {
      screenshot = argv[++i];
    } else if (argument == "--screenshot-sim" && i + 1 < argc) {
      screenshot_sim = argv[++i];
    } else if (argument == "--duration" && i + 1 < argc) {
      duration = std::stod(argv[++i]);
    } else if (argument == "--zoom" && i + 1 < argc) {
      zoom = std::stod(argv[++i]);
    } else if (argument == "--server" && i + 1 < argc) {
      server_port = std::stoi(argv[++i]);
    } else if (argument == "--help") {
      std::cout << "usage: krosc_turtlebot3 [--screenshot PATH] [--screenshot-sim PATH] [--duration SEC] [--headless] [--no-noise] [--zoom DIST] [--server PORT]\n";
      return 0;
    }
  }
  if (duration <= 0.0) {
    duration = (screenshot.empty() && screenshot_sim.empty() && !headless) ? -1.0 : 4.0;
  }

  auto time_source = std::make_shared<krosc::SimulatedTimeSource>();
  krosc::Context::Options options;
  options.time_source = time_source;
  options.name = "krosc_turtlebot3";
  if (server_port > 0) {
    // 원격 Viz는 PeerNetwork로 붙는다 (net::Server 아님).
    options.network.name = "krosc_turtlebot3";
    options.network.host = "0.0.0.0";
    options.network.port = static_cast<std::uint16_t>(server_port);
  }
  krosc::Context context(options);
  if (const auto* peer = context.peer_network()) {
    std::printf("[krosc] network peer listen: 0.0.0.0:%d\n", static_cast<int>(peer->port()));
  }

  krosc::sim::MazeWorld world(10, 42);
  auto robot_holder = std::make_unique<krosc::sim::TurtleBot3>(noisy, 42);
  auto* robot = static_cast<krosc::sim::TurtleBot3*>(world.add(std::move(robot_holder)));

  const std::string urdf = read_text(KROSC_TURTLEBOT3_URDF);
  if (urdf.empty()) {
    std::cerr << "URDF를 읽지 못했다: " << KROSC_TURTLEBOT3_URDF << "\n";
  }

  krosc::examples::RobotBridge bridge(context, *robot);
  krosc::examples::MapPublisher map_publisher(context, world.walls());
  krosc::nodes::RobotStatePublisher robot_state(context, "robot_state_publisher",
                                                {{"robot_description", urdf}});
  krosc::nodes::DiffDriveOdometry odometry(context, "diff_drive_odometry",
                                           {{"publish_tf", false},
                                            {"wheel_radius", krosc::sim::TurtleBot3::kWheelRadius},
                                            {"wheel_separation", krosc::sim::TurtleBot3::kWheelSeparation}});
  krosc::nodes::Ekf ekf(context, "ekf_filter_node",
                        {{"odom0", std::string("odom")},
                         {"odom0_config", ekf_config({6, 11})},
                         {"imu0", std::string("imu")},
                         {"imu0_config", ekf_config({11})},
                         {"publish_tf", true}});

  krosc::sim::SnapshotRecorder recorder(world, *robot);
  krosc::sim::SimWindow sim_window(world.walls(), *robot, recorder.store());
  krosc::viz::VizWindow window(context);
  window.set_fixed_frame("map");
  if (zoom > 0.0) {
    window.scene().set_camera_distance(zoom);
    sim_window.view().set_camera_distance(zoom);
  }
  sim_window.resize(740, 860);
  sim_window.move(20, 20);
  window.resize(1100, 860);
  window.move(780, 20);
  sim_window.show();
  window.show();
  bridge.logger().info("TurtleBot3 demo: noisy=%s, pucks=%d, nodes=%d", noisy ? "true" : "false",
                       static_cast<int>(world.pucks().size()),
                       static_cast<int>(context.node_names().size()));

  krosc::sim::SimulationRunner runner(world, 60.0);
  runner.add_listener([&](krosc::sim::World& current) {
    time_source->set(current.time());
    recorder.record();
  });
  if (screenshot.empty() && duration <= 0.0) {
    // 사람이 조종한다. 시작하자마자 벽을 보고 있도록 살짝 돌려 둔다.
    robot->set_command(krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  } else {
    robot->set_command(krosc::msgs::Twist{{0.16, 0.0, 0.0}, {0.0, 0.0, 0.28}});
  }
  runner.start();

  if (!screenshot.empty() || !screenshot_sim.empty()) {
    for (const auto& path_text : {screenshot, screenshot_sim}) {
      if (!path_text.empty()) {
        const std::filesystem::path path(path_text);
        if (path.has_parent_path()) {
          std::filesystem::create_directories(path.parent_path());
        }
      }
    }
    QTimer::singleShot(static_cast<int>(duration * 1000.0), &window, [&] {
      bool saved = true;
      if (!screenshot.empty()) {
        const bool ok = window.save_screenshot(QString::fromStdString(screenshot));
        std::cout << (ok ? "screenshot saved: " : "screenshot failed: ") << screenshot << "\n";
        saved = saved && ok;
      }
      if (!screenshot_sim.empty()) {
        const bool ok = sim_window.save_screenshot(QString::fromStdString(screenshot_sim));
        std::cout << (ok ? "sim screenshot saved: " : "sim screenshot failed: ") << screenshot_sim << "\n";
        saved = saved && ok;
      }
      application.exit(saved ? 0 : 2);
    });
  } else if (duration > 0.0) {
    QTimer::singleShot(static_cast<int>(duration * 1000.0), &application, [&] { application.quit(); });
  }

  const int result = application.exec();
  runner.stop();
  context.shutdown();
  return result;
}
