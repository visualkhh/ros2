#include "krosc/viz/viz.hpp"

#include <QApplication>
#include <QTimer>

#include <cstdio>
#include <utility>

#include "krosc/viz/viz_window.hpp"

namespace krosc::viz {

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

net::PeerNetworkOptions peer_network_options(const VizOptions& options) {
  net::PeerNetworkOptions network;
  network.name = "krosc_viz";
  if (!options.server.empty()) {
    const ServerAddress address = parse_server(options.server);
    network.add_server(address.host, address.port);
  }
  return network;
}

Viz::Viz(Context& context, VizOptions options, VizPluginRegistry registry)
    : context_(&context), options_(std::move(options)), registry_(std::move(registry)) {}

Viz::~Viz() {
  if (status_timer_ != nullptr) {
    status_timer_->stop();
  }
  window_.reset();
}

bool Viz::initialize(std::string& error) {
  if (initialized_) {
    return true;
  }
  window_ = std::make_unique<VizWindow>(*context_);
  window_->set_fixed_frame(options_.fixed_frame);
  window_->load_plugins(registry_, options_.plugins);
  window_->set_tab(options_.tab);
  if (options_.distance > 0.0) {
    window_->set_camera_distance(options_.distance);
  }
  update_connection_status();
  window_->show();

  status_timer_ = new QTimer(window_.get());
  QObject::connect(status_timer_, &QTimer::timeout, window_.get(),
                   [this] { update_connection_status(); });
  status_timer_->start(1000);

  if (!options_.screenshot.empty()) {
    QTimer::singleShot(static_cast<int>(options_.duration * 1000.0), window_.get(), [this] {
      const bool saved = window_->save_screenshot(QString::fromStdString(options_.screenshot));
      std::printf("%s%s\n", saved ? "screenshot saved: " : "screenshot failed: ",
                  options_.screenshot.c_str());
      QApplication::exit(saved ? 0 : 2);
    });
  }
  initialized_ = true;
  error.clear();
  return true;
}

void Viz::update_connection_status() {
  if (window_ == nullptr || context_ == nullptr) {
    return;
  }
  bool connected = true;
  std::string endpoint = "local";
  if (const auto* peer = context_->peer_network()) {
    const auto endpoints = peer->peer_endpoints();
    connected = !endpoints.empty();
    if (endpoints.empty()) {
      endpoint = "peer: waiting";
    } else {
      endpoint = "peer: ";
      for (std::size_t i = 0; i < endpoints.size(); ++i) {
        if (i != 0) {
          endpoint += ",";
        }
        endpoint += endpoints[i];
      }
    }
  }
  window_->set_connection_status(connected, endpoint);
}

int Viz::run() {
  if (!initialized_) {
    std::string error;
    if (!initialize(error)) {
      std::fprintf(stderr, "[viz] %s\n", error.c_str());
      return 1;
    }
  }
  return QApplication::exec();
}

}  // namespace krosc::viz
