#include <QApplication>
#include <QTimer>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "krosc/krosc.hpp"
#include "krosc/msgs.hpp"
#include "krosc/nodes/diff_drive_odometry.hpp"
#include "krosc/nodes/ekf.hpp"
#include "krosc/nodes/robot_state_publisher.hpp"
#include "krosc/qos.hpp"
#include "krosc/simulator/objects/turtlebot3.hpp"
#include "krosc/simulator/simulator.hpp"
#include "krosc/simulator/view_window.hpp"
#include "krosc/simulator/worlds/maze_world.hpp"
#include "krosc/time/time.hpp"
#include "krosc/viz/viz_window.hpp"

// 데모 앱 로컬 ROS 접착제 (코어/에셋 라이브러리에 넣지 않는다).
#include "map_publisher.hpp"
#include "robot_bridge.hpp"

#ifndef KROSC_SIMULATOR_URDF
#define KROSC_SIMULATOR_URDF "description/turtlebot3_waffle_pi.urdf"
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
  for (const int index : enabled) {
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
  int pucks = 10;
  std::uint32_t seed = 42;
  double duration = 0.0;
  double zoom = 0.0;
  int peer_port = 0;
  std::string screenshot;
  std::string screenshot_sim;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--headless") {
      headless = true;
    } else if (argument == "--no-noise") {
      noisy = false;
    } else if (argument == "--pucks" && i + 1 < argc) {
      pucks = std::stoi(argv[++i]);
    } else if (argument == "--seed" && i + 1 < argc) {
      seed = static_cast<std::uint32_t>(std::stoul(argv[++i]));
    } else if (argument == "--screenshot" && i + 1 < argc) {
      screenshot = argv[++i];
    } else if (argument == "--screenshot-sim" && i + 1 < argc) {
      screenshot_sim = argv[++i];
    } else if (argument == "--duration" && i + 1 < argc) {
      duration = std::stod(argv[++i]);
    } else if (argument == "--zoom" && i + 1 < argc) {
      zoom = std::stod(argv[++i]);
    } else if (argument == "--server" && i + 1 < argc) {
      peer_port = std::stoi(argv[++i]);
    } else if (argument == "--help") {
      std::cout << "usage: krosc_simulator [--screenshot PATH] [--screenshot-sim PATH]\n"
                   "                       [--duration SEC] [--headless] [--no-noise]\n"
                   "                       [--pucks N] [--seed N] [--zoom DIST] [--server PORT]\n";
      return 0;
    }
  }
  if (duration <= 0.0) {
    duration = (screenshot.empty() && screenshot_sim.empty() && !headless) ? -1.0 : 4.0;
  }

  auto time_source = std::make_shared<krosc::SimulatedTimeSource>();
  krosc::Context::Options options;
  options.time_source = time_source;
  options.name = "krosc_simulator";
  if (peer_port > 0) {
    options.network.name = "krosc_simulator";
    options.network.host = "0.0.0.0";
    options.network.port = static_cast<std::uint16_t>(peer_port);
  }
  krosc::Krosc app(nullptr, options);
  if (const auto* peer = app.context().peer_network()) {
    std::printf("[krosc] peer listen: 0.0.0.0:%d\n", static_cast<int>(peer->port()));
  }

  auto world = std::make_unique<krosc::simulator::MazeWorld>(pucks, seed);
  auto robot_holder = std::make_unique<krosc::simulator::TurtleBot3>(noisy, seed);
  auto* robot = static_cast<krosc::simulator::TurtleBot3*>(
      world->add(std::move(robot_holder)));

  const std::string urdf = read_text(KROSC_SIMULATOR_URDF);
  if (urdf.empty()) {
    std::cerr << "URDF를 읽지 못했다: " << KROSC_SIMULATOR_URDF << "\n";
    return 1;
  }
  krosc::simulator::MapPublisher map_publisher(app.context(), world->walls());
  krosc::simulator::RobotBridge bridge(app.context(), *robot);
  krosc::nodes::RobotStatePublisher robot_state(
      app.context(), "robot_state_publisher",
      std::map<std::string, krosc::ParamValue>{{"robot_description", urdf}});
  krosc::nodes::DiffDriveOdometry odometry(
      app.context(), "diff_drive_odometry",
      std::map<std::string, krosc::ParamValue>{
          {"publish_tf", false},
          {"wheel_radius", krosc::simulator::TurtleBot3::kWheelRadius},
          {"wheel_separation", krosc::simulator::TurtleBot3::kWheelSeparation}});
  krosc::nodes::Ekf ekf(
      app.context(), "ekf_filter_node",
      std::map<std::string, krosc::ParamValue>{
          {"odom0", std::string("odom")},
          {"odom0_config", ekf_config({6, 11})},
          {"imu0", std::string("imu")},
          {"imu0_config", ekf_config({11})},
          {"publish_tf", true}});

  krosc::simulator::Simulator simulator(app.context(), *world, *robot);
  krosc::simulator::SimWindow sim_window(world->walls(), simulator.snapshots(), bridge);
  krosc::viz::VizWindow window(app.context());
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
  bridge.logger().info("Simulator: noisy=%s, pucks=%zu, nodes=%zu",
                       noisy ? "true" : "false", world->pucks().size(),
                       app.context().node_names().size());

  const auto send_twist = [&bridge](double linear, double angular) {
    bridge.publish("cmd_vel",
                   krosc::msgs::Twist{{linear, 0.0, 0.0}, {0.0, 0.0, angular}},
                   krosc::QOS_COMMAND);
  };
  if (screenshot.empty() && duration <= 0.0) {
    send_twist(0.0, 0.0);
  } else {
    send_twist(0.16, 0.28);
  }
  simulator.start();

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
        std::cout << (ok ? "sim screenshot saved: " : "sim screenshot failed: ")
                  << screenshot_sim << "\n";
        saved = saved && ok;
      }
      application.exit(saved ? 0 : 2);
    });
  } else if (duration > 0.0) {
    QTimer::singleShot(static_cast<int>(duration * 1000.0), &application,
                       [&] { application.quit(); });
  }

  const int result = application.exec();
  simulator.stop();
  return result;
}
