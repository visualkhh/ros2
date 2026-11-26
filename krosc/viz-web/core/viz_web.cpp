#include "krosc/viz_web/viz_web.hpp"

#include <algorithm>
#include <csignal>
#include <set>

#include <chrono>
#include <cstdio>
#include <thread>
#include <utility>

#include "krosc/net/net.hpp"
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

QoS qos_from_name(const std::string& name) {
  if (name == "sensor" || name == "sensor_data") return QOS_SENSOR_DATA;
  if (name == "latched" || name == "transient_local") return QOS_LATCHED;
  if (name == "command") return QOS_COMMAND;
  return QOS_DEFAULT;
}

std::string qos_name(const QoS& qos) {
  if (qos.durability == Durability::TransientLocal) return "latched";
  if (qos.depth.has_value() && *qos.depth == 5) return "sensor";
  if (qos.depth.has_value() && *qos.depth == 1) return "command";
  return "default";
}

}  // namespace

VizWeb::VizWeb(VizWebOptions options, VizWebPluginRegistry registry, Context& context)
    : options_(std::move(options)), registry_(std::move(registry)), context_(&context),
      rpc_(std::make_unique<WebRpc>(context)) {
  register_default_web_rpc(*rpc_, context);
  register_topic_rpc();
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
    robot_connected_.store(connected);
    std::printf("[viz-web] robot %s: %s\n", connected ? "connected" : "disconnected",
                link_->endpoint().c_str());
    std::fflush(stdout);
    if (!connected) {
      // 늦게 접속한 브라우저가 끊기기 전 지도를 다시 받지 않게 한다.
      server_->clear_cache();
    }
    server_->broadcast_text(std::string("{\"type\":\"robot\",\"connected\":") +
                            (connected ? "true" : "false") + "}");
  });
  // 늦게 붙은 브라우저도 현재 로봇 연결 상태를 바로 받는다.
  server_->set_text_on_connect([this] {
    return std::string("{\"type\":\"robot\",\"connected\":") +
           (robot_connected_.load() ? "true" : "false") + "}";
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
    std::set<std::string> extra_names;
    for (const auto& extra : options_.forward_topics) {
      extra_names.insert(extra.name);
    }
    for (const auto& topic : topics) {
      const std::string source = extra_names.count(topic.name) != 0 ? "project" : "core";
      std::printf("[viz-web] forward %s (kind=%u, source=%s)\n", topic.name.c_str(),
                  static_cast<unsigned>(topic.kind), source.c_str());
      std::string topic_error;
      if (!register_topic(topic, source, true, topic_error)) {
        std::fprintf(stderr, "[viz-web] subscribe %s 실패: %s\n", topic.name.c_str(),
                     topic_error.c_str());
      }
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

void VizWeb::register_topic_rpc() {
  rpc_->add("topic/list", [this](const Json&, const WebRpc::Notify&) { return topic_list_json(); });

  rpc_->add("topic/register", [this](const Json& params, const WebRpc::Notify&) {
    ContextLink::TopicSpec spec;
    spec.name = params["name"].as_string();
    spec.kind = static_cast<net::Kind>(params["kind"].as_int(static_cast<int>(net::Kind::String)));
    spec.qos = qos_from_name(params["qos"].as_string("default"));
    const std::string source = params["source"].as_string("plugin");
    const bool subscribe = params["subscribe"].as_bool(true);
    std::string error;
    const bool ok = register_topic(spec, source, subscribe, error);
    Json result = Json::object();
    result["ok"] = ok;
    result["subscribed"] = ok && subscribe;
    if (!error.empty()) {
      result["error"] = error;
    }
    return result;
  });

  rpc_->add("topic/subscribe", [this](const Json& params, const WebRpc::Notify&) {
    std::string error;
    const bool ok = set_topic_subscription(params["name"].as_string(), true, error);
    Json result = Json::object();
    result["ok"] = ok;
    if (!error.empty()) {
      result["error"] = error;
    }
    return result;
  });

  rpc_->add("topic/unsubscribe", [this](const Json& params, const WebRpc::Notify&) {
    std::string error;
    const bool ok = set_topic_subscription(params["name"].as_string(), false, error);
    Json result = Json::object();
    result["ok"] = ok;
    if (!error.empty()) {
      result["error"] = error;
    }
    return result;
  });
}

bool VizWeb::register_topic(const ContextLink::TopicSpec& spec, const std::string& source,
                            bool subscribe, std::string& error) {
  if (spec.name.empty()) {
    error = "topic 이름이 비었다";
    return false;
  }
  std::lock_guard<std::mutex> lock(topics_mutex_);
  if (!link_) {
    error = "VizWeb가 초기화되지 않았다";
    return false;
  }
  auto found = topics_.find(spec.name);
  if (found == topics_.end()) {
    TopicEntry entry;
    entry.spec = spec;
    entry.source = source.empty() ? "plugin" : source;
    found = topics_.emplace(spec.name, std::move(entry)).first;
  } else {
    if (found->second.subscribed && found->second.spec.kind != spec.kind) {
      error = "이미 다른 kind로 구독 중이다";
      return false;
    }
    if (!found->second.subscribed) {
      found->second.spec.kind = spec.kind;
      found->second.spec.qos = spec.qos;
    }
    if (!source.empty()) {
      found->second.source = source;
    }
  }
  if (subscribe && !found->second.subscribed) {
    try {
      if (!link_->subscribe(found->second.spec)) {
        error = "지원하지 않는 메시지 kind이거나 이미 구독 중이다";
        return false;
      }
      found->second.subscribed = true;
    } catch (const std::exception& exception) {
      error = exception.what();
      return false;
    }
  }
  return true;
}

bool VizWeb::set_topic_subscription(const std::string& name, bool subscribe,
                                    std::string& error) {
  std::lock_guard<std::mutex> lock(topics_mutex_);
  if (!link_) {
    error = "VizWeb가 초기화되지 않았다";
    return false;
  }
  auto found = topics_.find(name);
  if (found == topics_.end()) {
    error = "등록되지 않은 토픽이다";
    return false;
  }
  if (found->second.subscribed == subscribe) {
    return true;
  }
  try {
    if (subscribe) {
      if (!link_->subscribe(found->second.spec)) {
        error = "구독에 실패했다";
        return false;
      }
    } else if (!link_->unsubscribe(name)) {
      error = "구독 해제에 실패했다";
      return false;
    }
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
  found->second.subscribed = subscribe;
  return true;
}

Json VizWeb::topic_list_json() const {
  Json array = Json::array();
  {
    std::lock_guard<std::mutex> lock(topics_mutex_);
    for (const auto& [name, entry] : topics_) {
      Json item = Json::object();
      item["name"] = name;
      item["kind"] = static_cast<int>(entry.spec.kind);
      item["source"] = entry.source;
      item["subscribed"] = entry.subscribed;
      item["qos"] = qos_name(entry.spec.qos);
      array.push(std::move(item));
    }
  }
  Json result = Json::object();
  result["topics"] = std::move(array);
  return result;
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
  {
    std::lock_guard<std::mutex> lock(topics_mutex_);
    topics_.clear();
  }
  link_.reset();
  initialized_ = false;
}

std::uint16_t VizWeb::port() const { return server_ ? server_->port() : 0; }

bool VizWeb::connected() const { return link_ && link_->connected(); }

}  // namespace krosc::viz_web
