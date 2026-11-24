#include <csignal>

#include <atomic>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>

#include "krosc/web/http_server.hpp"
#include "krosc/web/robot_link.hpp"

#ifndef KROSC_WEB_ROOT
#define KROSC_WEB_ROOT "web/www"
#endif

namespace {

std::atomic<bool> g_stop{false};

void stop_handler(int) { g_stop.store(true); }

struct RobotAddress {
  std::string host = "127.0.0.1";
  std::uint16_t port = 9000;
};

RobotAddress parse_robot(const std::string& text) {
  RobotAddress address;
  const auto colon = text.rfind(':');
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

void usage() {
  std::printf("usage: krosc_web [--robot HOST:PORT] [--port PORT] [--bind ADDRESS] [--www DIR]\n");
}

}  // namespace

int main(int argc, char** argv) {
  RobotAddress robot;
  std::uint16_t port = 8080;
  std::string bind_address = "0.0.0.0";
  std::string www = KROSC_WEB_ROOT;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--robot" && index + 1 < argc) {
      robot = parse_robot(argv[++index]);
    } else if (argument == "--port" && index + 1 < argc) {
      port = static_cast<std::uint16_t>(std::stoi(argv[++index]));
    } else if (argument == "--bind" && index + 1 < argc) {
      bind_address = argv[++index];
    } else if (argument == "--www" && index + 1 < argc) {
      www = argv[++index];
    } else if (argument == "--help" || argument == "-h") {
      usage();
      return 0;
    } else {
      usage();
      return 1;
    }
  }

  std::signal(SIGINT, stop_handler);
  std::signal(SIGTERM, stop_handler);

  krosc::web::RobotLink link(robot.host, robot.port);
  krosc::web::HttpServer server(bind_address, port, www, link);
  link.set_frame_handler([&server](const std::vector<std::uint8_t>& body) {
    server.broadcast_binary(body);
  });
  link.set_status_handler([&server](bool connected) {
    server.broadcast_text(std::string("{\"type\":\"robot\",\"connected\":") +
                          (connected ? "true" : "false") + "}");
  });
  link.start();
  try {
    server.start();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "krosc_web: %s\n", error.what());
    link.shutdown();
    return 1;
  }

  std::printf("[web] http://%s:%u/  (robot %s:%u)\n",
              bind_address == "0.0.0.0" ? "localhost" : bind_address.c_str(),
              static_cast<unsigned>(server.port()), robot.host.c_str(),
              static_cast<unsigned>(robot.port));
  std::fflush(stdout);

  while (!g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  link.shutdown();
  return 0;
}
