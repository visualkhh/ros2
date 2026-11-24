#include <QApplication>
#include <QTimer>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/net.hpp"
#include "krosc/time.hpp"
#include "krosc/viz/viz_plugin.hpp"
#include "krosc/viz/viz_window.hpp"

#include "plugins/moveit_plugin.hpp"
#include "plugins/project_panel.hpp"
#include "plugins/rl_nav_plugin.hpp"

namespace {

struct ServerAddress {
  std::string host;
  std::uint16_t port = 9000;
};

ServerAddress parse_server(const std::string& text) {
  ServerAddress address;
  const std::size_t colon = text.rfind(':');
  if (colon == std::string::npos) {
    address.host = text;
    return address;
  }
  address.host = text.substr(0, colon);
  try {
    address.port = static_cast<std::uint16_t>(std::stoi(text.substr(colon + 1)));
  } catch (...) {
    address.port = 9000;
  }
  return address;
}

std::vector<std::string> split_list(const std::string& text) {
  std::vector<std::string> result;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const std::size_t begin = item.find_first_not_of(" \t");
    const std::size_t end = item.find_last_not_of(" \t");
    if (begin != std::string::npos) {
      result.push_back(item.substr(begin, end - begin + 1));
    }
  }
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  QApplication application(argc, argv);
  std::string screenshot;
  std::string server_text;
  std::string fixed_frame = "map";
  std::string tab = "3d";
  std::string plugins_text = "project_panel,moveit,rl_nav";
  double duration = 2.0;
  double distance = 0.0;
  bool teleop_panel = false;
  bool manipulator_panel = false;
  bool moveit_plugin = true;
  bool moveit_override = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--screenshot" && i + 1 < argc) {
      screenshot = argv[++i];
    } else if (argument == "--duration" && i + 1 < argc) {
      duration = std::stod(argv[++i]);
    } else if (argument == "--server" && i + 1 < argc) {
      server_text = argv[++i];
    } else if (argument == "--fixed-frame" && i + 1 < argc) {
      fixed_frame = argv[++i];
    } else if (argument == "--tab" && i + 1 < argc) {
      tab = argv[++i];
    } else if (argument == "--plugins" && i + 1 < argc) {
      plugins_text = argv[++i];
    } else if (argument == "--distance" && i + 1 < argc) {
      distance = std::stod(argv[++i]);
    } else if (argument == "--manual") {
      teleop_panel = true;
      manipulator_panel = true;
    } else if (argument == "--teleop") {
      teleop_panel = true;
      manipulator_panel = false;
    } else if (argument == "--manipulator") {
      teleop_panel = false;
      manipulator_panel = true;
    } else if (argument == "--moveit") {
      moveit_plugin = true;
      moveit_override = true;
    } else if (argument == "--no-moveit") {
      moveit_plugin = false;
      moveit_override = true;
    } else if (argument == "--no-manual") {
      teleop_panel = false;
      manipulator_panel = false;
    } else if (argument == "--no-plugins") {
      plugins_text.clear();
      moveit_plugin = false;
      moveit_override = true;
    } else if (argument == "--help") {
      std::printf(
          "usage: krosc_ws_viz_app [--server HOST[:PORT]] [--fixed-frame FRAME] [--tab 3d|map|moveit]\n"
          "       [--plugins a,b,...] [--manual|--teleop|--manipulator|--moveit|--no-manual] [--no-plugins]\n"
          "       [--distance M] [--screenshot PATH] [--duration SEC]\n");
      return 0;
    }
  }

  krosc::Context::Options options;
  options.time_source = std::make_shared<krosc::SystemTimeSource>();
  krosc::Context context(options);

  std::unique_ptr<krosc::net::Client> client;
  if (!server_text.empty()) {
    const ServerAddress address = parse_server(server_text);
    client = std::make_unique<krosc::net::Client>(context, address.host, address.port);
  }

  krosc::viz::VizPluginRegistry registry;
  registry.add("project_panel", [] { return std::make_unique<krosc_ws::viz::ProjectPanelPlugin>(); });
  registry.add("moveit", [] { return std::make_unique<krosc_ws::viz::MoveItPlugin>(); });
  registry.add("rl_nav", [] { return std::make_unique<krosc_ws::viz::RlNavPlugin>(); });

  std::vector<std::string> plugins = split_list(plugins_text);
  if (moveit_override) {
    if (moveit_plugin) {
      if (std::find(plugins.begin(), plugins.end(), "moveit") == plugins.end()) {
        plugins.push_back("moveit");
      }
    } else {
      plugins.erase(std::remove(plugins.begin(), plugins.end(), "moveit"), plugins.end());
    }
  }

  krosc::viz::VizWindow window(context);
  window.set_fixed_frame(fixed_frame);
  window.set_manual_panels(teleop_panel, manipulator_panel);
  window.load_plugins(registry, plugins);
  window.set_tab(tab);
  if (distance > 0.0) {
    window.set_camera_distance(distance);
  }
  if (client) {
    window.set_connection_status(client->connected(), client->endpoint());
  }
  window.show();

  QTimer status_timer;
  if (client) {
    QObject::connect(&status_timer, &QTimer::timeout, &window,
                     [&] { window.set_connection_status(client->connected(), client->endpoint()); });
    status_timer.start(1000);
  }

  if (!screenshot.empty()) {
    QTimer::singleShot(static_cast<int>(duration * 1000.0), &window, [&] {
      const bool saved = window.save_screenshot(QString::fromStdString(screenshot));
      std::printf("%s%s\n", saved ? "screenshot saved: " : "screenshot failed: ", screenshot.c_str());
      application.exit(saved ? 0 : 2);
    });
  }
  const int result = application.exec();
  status_timer.stop();
  client.reset();
  context.shutdown();
  return result;
}
