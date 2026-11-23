#include <QApplication>
#include <QTimer>

#include <chrono>
#include <memory>
#include <string>

#include "krosc/context.hpp"
#include "krosc/net.hpp"
#include "krosc/time.hpp"
#include "krosc/viz/viz_window.hpp"

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
}  // namespace

int main(int argc, char** argv) {
  QApplication application(argc, argv);
  std::string screenshot;
  std::string server_text;
  std::string fixed_frame = "odom";
  double duration = 2.0;
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
    } else if (argument == "--help") {
      std::printf("usage: krosc_viz_app [--server HOST[:PORT]] [--fixed-frame FRAME] [--screenshot PATH] [--duration SEC]\n");
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
  krosc::viz::VizWindow window(context);
  window.set_fixed_frame(fixed_frame);
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
