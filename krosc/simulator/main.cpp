#include <QApplication>
#include <QTimer>

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include "krosc/krosc.hpp"
#include "krosc/simulator/simulator.hpp"
#include "krosc/simulator/view_window.hpp"
#include "krosc/time/time.hpp"
#include "krosc/viz/viz_window.hpp"

#ifndef KROSC_SIMULATOR_URDF
#define KROSC_SIMULATOR_URDF "description/turtlebot3_waffle_pi.urdf"
#endif

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

  krosc::simulator::Simulator simulator(
      app.context(),
      krosc::simulator::SimulatorOptions{noisy, pucks, seed, 100.0, KROSC_SIMULATOR_URDF});

  krosc::simulator::SimWindow sim_window(simulator.world().walls(), simulator.robot(),
                                         simulator.snapshots());
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
  simulator.bridge_node().logger().info(
      "Simulator: noisy=%s, pucks=%zu, nodes=%zu", noisy ? "true" : "false",
      simulator.maze_world().pucks().size(), app.context().node_names().size());

  if (screenshot.empty() && duration <= 0.0) {
    simulator.set_stop_command();
  } else {
    simulator.set_demo_command();
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
