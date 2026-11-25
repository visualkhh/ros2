#include "krosc/viz_web/viz_web.hpp"

#include <csignal>

#include <chrono>
#include <cstdio>
#include <thread>
#include <utility>

#include "krosc/viz_web/http_server.hpp"
#include "krosc/viz_web/robot_link.hpp"

namespace krosc::viz_web {

namespace {

std::atomic<bool> g_stop{false};

void stop_handler(int) { g_stop.store(true); }

}  // namespace

VizWeb::VizWeb(VizWebOptions options, VizWebPluginRegistry registry)
    : options_(std::move(options)), registry_(std::move(registry)) {}

VizWeb::~VizWeb() { shutdown(); }

bool VizWeb::initialize(std::string& error) {
  if (initialized_) {
    return true;
  }
  link_ = std::make_unique<RobotLink>(options_.robot_host, options_.robot_port);
  server_ = std::make_unique<HttpServer>(options_.bind_address, options_.port, options_.www_root,
                                         *link_,
                                         [this] { return registry_.manifest_json(); });
  link_->set_frame_handler([this](const std::vector<std::uint8_t>& body) {
    server_->broadcast_binary(body);
  });
  link_->set_status_handler([this](bool connected) {
    server_->broadcast_text(std::string("{\"type\":\"robot\",\"connected\":") +
                            (connected ? "true" : "false") + "}");
  });
  link_->start();
  try {
    server_->start();
  } catch (const std::exception& exception) {
    error = exception.what();
    link_->shutdown();
    server_.reset();
    link_.reset();
    return false;
  }
  initialized_ = true;
  error.clear();
  return true;
}

int VizWeb::run() {
  std::string error;
  if (!initialize(error)) {
    std::fprintf(stderr, "krosc_viz_web: %s\n", error.c_str());
    return 1;
  }

  g_stop.store(false);
  std::signal(SIGINT, stop_handler);
  std::signal(SIGTERM, stop_handler);
  std::printf("[viz-web] http://%s:%u/  (robot %s:%u)\n",
              options_.bind_address == "0.0.0.0" ? "localhost" : options_.bind_address.c_str(),
              static_cast<unsigned>(server_->port()), options_.robot_host.c_str(),
              static_cast<unsigned>(options_.robot_port));
  std::fflush(stdout);

  while (!stop_.load() && !g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  shutdown();
  return 0;
}

void VizWeb::stop() { stop_.store(true); }

void VizWeb::shutdown() {
  if (link_) {
    link_->shutdown();
  }
  server_.reset();
  link_.reset();
  initialized_ = false;
}

std::uint16_t VizWeb::port() const { return server_ ? server_->port() : 0; }

bool VizWeb::connected() const { return link_ && link_->connected(); }

}  // namespace krosc::viz_web
