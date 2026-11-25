#include "krosc/viz_web/viz_web.hpp"

#include <algorithm>
#include <csignal>

#include <chrono>
#include <cstdio>
#include <thread>
#include <utility>

#include "krosc/net.hpp"
#include "krosc/viz_web/http_server.hpp"

namespace krosc::viz_web {

namespace {

std::atomic<bool> g_stop{false};

void stop_handler(int) { g_stop.store(true); }

std::vector<ContextLink::TopicSpec> default_forward_topics() {
  using net::Kind;
  std::vector<ContextLink::TopicSpec> topics{
      {"tf", Kind::TF, QOS_TF},
      {"tf_static", Kind::TF, QOS_TF_STATIC},
      {"robot_description", Kind::String, QOS_LATCHED},
      {"map", Kind::Grid, QOS_LATCHED},
      {"global_costmap/costmap", Kind::Grid, QOS_LATCHED},
      {"local_costmap/costmap", Kind::Grid, QOS_LATCHED},
      {"plan", Kind::Path, QOS_LATCHED},
      {"local_plan", Kind::Path, QOS_SENSOR_DATA},
      {"scan", Kind::Scan, QOS_SENSOR_DATA},
      {"imu", Kind::Imu, QOS_SENSOR_DATA},
      {"odom", Kind::Odometry, QOS_SENSOR_DATA},
      {"odometry/filtered", Kind::Odometry, QOS_SENSOR_DATA},
      {"joint_states", Kind::JointState, QOS_DEFAULT},
      {"diagnostics", Kind::Diagnostics, QOS_DEFAULT},
      {"rosout", Kind::Log, QOS_DEFAULT},
      {"map_list", Kind::String, QOS_LATCHED},
      {"map_name", Kind::String, QOS_LATCHED},
      {"map_status", Kind::String, QOS_LATCHED},
      {"slam_status", Kind::String, QOS_LATCHED},
      {"goal_status", Kind::String, QOS_LATCHED},
      {"collision_status", Kind::String, QOS_LATCHED},
  };
  return topics;
}

}  // namespace

VizWeb::VizWeb(VizWebOptions options, VizWebPluginRegistry registry, Context& context)
    : options_(std::move(options)), registry_(std::move(registry)), context_(&context),
      rpc_(std::make_unique<WebRpc>(context)) {
  register_default_web_rpc(*rpc_, context);
}

VizWeb::~VizWeb() { shutdown(); }

bool VizWeb::initialize(std::string& error) {
  if (initialized_) {
    return true;
  }
  if (context_ == nullptr) {
    error = "Context가 없다";
    return false;
  }
  link_ = std::make_unique<ContextLink>(*context_);
  rpc_->start();
  std::vector<std::string> roots;
  if (!options_.overlay_root.empty()) {
    roots.push_back(options_.overlay_root);
  }
  roots.push_back(options_.www_root);
  server_ = std::make_unique<HttpServer>(
      options_.bind_address, options_.port, std::move(roots),
      [this](const net::Frame& frame) {
        if (link_) {
          (void)link_->send(frame);
        }
      },
      [this] { return registry_.manifest_json(); },
      [this](const std::string& text, HttpServer::TextReply reply) {
        rpc_->handle(text, std::move(reply));
      });
  link_->set_frame_handler([this](const net::Frame& frame) {
    server_->broadcast_binary(net::encode_body(frame));
  });
  link_->set_status_handler([this](bool connected) {
    std::printf("[viz-web] robot %s: %s\n", connected ? "connected" : "disconnected",
                link_->endpoint().c_str());
    std::fflush(stdout);
    server_->broadcast_text(std::string("{\"type\":\"robot\",\"connected\":") +
                            (connected ? "true" : "false") + "}");
  });
  try {
    server_->start();
    auto topics = default_forward_topics();
    for (const auto& extra : options_.forward_topics) {
      const auto found = std::find_if(topics.begin(), topics.end(),
                                      [&](const ContextLink::TopicSpec& topic) {
                                        return topic.name == extra.name;
                                      });
      if (found == topics.end()) {
        topics.push_back(extra);
      } else {
        *found = extra;
      }
    }
    for (const auto& topic : topics) {
      link_->subscribe(topic);
    }
    link_->start();
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
    link_->set_frame_handler({});
    link_->set_status_handler({});
    link_->shutdown();
  }
  server_.reset();
  rpc_->shutdown();
  link_.reset();
  initialized_ = false;
}

std::uint16_t VizWeb::port() const { return server_ ? server_->port() : 0; }

bool VizWeb::connected() const { return link_ && link_->connected(); }

}  // namespace krosc::viz_web
