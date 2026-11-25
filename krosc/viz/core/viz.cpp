#include "krosc/viz/viz.hpp"

#include <QApplication>
#include <QTimer>

#include <cstdio>
#include <utility>

#include "krosc/net.hpp"
#include "krosc/time.hpp"
#include "krosc/viz/viz_window.hpp"

namespace krosc::viz {

namespace {

Context::Options context_options() {
  Context::Options options;
  options.time_source = std::make_shared<SystemTimeSource>();
  return options;
}

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

Viz::Viz(VizOptions options, VizPluginRegistry registry)
    : options_(std::move(options)), registry_(std::move(registry)), context_(context_options()) {}

Viz::~Viz() {
  if (status_timer_ != nullptr) {
    status_timer_->stop();
  }
  client_.reset();
  window_.reset();
  context_.shutdown();
}

bool Viz::initialize(std::string& error) {
  if (initialized_) {
    return true;
  }
  if (!options_.server.empty()) {
    const ServerAddress address = parse_server(options_.server);
    try {
      client_ = std::make_unique<net::Client>(context_, address.host, address.port);
    } catch (const std::exception& exception) {
      error = std::string("network client: ") + exception.what();
      return false;
    }
  }

  window_ = std::make_unique<VizWindow>(context_);
  window_->set_fixed_frame(options_.fixed_frame);
  window_->load_plugins(registry_, options_.plugins);
  window_->set_tab(options_.tab);
  if (options_.distance > 0.0) {
    window_->set_camera_distance(options_.distance);
  }
  if (client_) {
    window_->set_connection_status(client_->connected(), client_->endpoint());
  }
  window_->show();

  status_timer_ = new QTimer(window_.get());
  if (client_) {
    QObject::connect(status_timer_, &QTimer::timeout, window_.get(), [this] {
      window_->set_connection_status(client_->connected(), client_->endpoint());
    });
    status_timer_->start(1000);
  }

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
