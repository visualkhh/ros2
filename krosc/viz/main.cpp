#include <QApplication>
#include <QTimer>

#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/net.hpp"
#include "krosc/time.hpp"
#include "krosc/viz/plugins/manipulator_plugin.hpp"
#include "krosc/viz/plugins/navigation_plugin.hpp"
#include "krosc/viz/plugins/slam_plugin.hpp"
#include "krosc/viz/plugins/teleop_plugin.hpp"
#include "krosc/viz/viz_plugin.hpp"
#include "krosc/viz/viz_window.hpp"

namespace {
struct ServerAddress {
  std::string host;
  std::uint16_t port = 9000;
};

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
}  // namespace

int main(int argc, char** argv) {
  QApplication application(argc, argv);
  std::string screenshot;
  std::string server_text;
  std::string fixed_frame = "odom";
  std::string tab = "3d";
  double duration = 2.0;
  double distance = 0.0;
  std::string plugins_text = "slam,navigation";
  bool teleop_panel = false;
  bool manipulator_panel = false;
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
    } else if (argument == "--distance" && i + 1 < argc) {
      distance = std::stod(argv[++i]);
    } else if (argument == "--plugins" && i + 1 < argc) {
      plugins_text = argv[++i];
    } else if (argument == "--manual") {
      teleop_panel = true;
      manipulator_panel = true;
    } else if (argument == "--teleop") {
      teleop_panel = true;
    } else if (argument == "--manipulator") {
      manipulator_panel = true;
    } else if (argument == "--no-manual") {
      teleop_panel = false;
      manipulator_panel = false;
    } else if (argument == "--no-plugins") {
      plugins_text.clear();
    } else if (argument == "--help") {
      std::printf("usage: krosc_viz_app [--server HOST[:PORT]] [--fixed-frame FRAME] [--tab 3d|map] [--plugins a,b,...] [--manual|--teleop|--manipulator|--no-manual] [--no-plugins] [--distance M] [--screenshot PATH] [--duration SEC]\n");
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
  registry.add("teleop", [] { return std::make_unique<krosc::viz::TeleopPlugin>(); });
  registry.add("slam", [] { return std::make_unique<krosc::viz::SlamPlugin>(); });
  registry.add("navigation", [] { return std::make_unique<krosc::viz::NavigationPlugin>(); });
  registry.add("manipulator", [] { return std::make_unique<krosc::viz::ManipulatorPlugin>(); });
  std::vector<std::string> plugins = split_list(plugins_text);
  const auto ensure = [&plugins](const std::string& id) {
    if (std::find(plugins.begin(), plugins.end(), id) == plugins.end()) {
      plugins.push_back(id);
    }
  };
  if (teleop_panel) ensure("teleop");
  if (manipulator_panel) ensure("manipulator");

  krosc::viz::VizWindow window(context);
  window.set_fixed_frame(fixed_frame);
  window.load_plugins(registry, plugins);
  window.set_tab(tab == "map" ? 1 : 0);
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
