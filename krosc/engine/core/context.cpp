#include "krosc/context.hpp"

#include <memory>
#include <utility>

namespace krosc {

Context::Context() : Context(Options{}) {}

Context::Context(Options options)
    : name_(std::move(options.name)),
      time_source_(options.time_source ? std::move(options.time_source) : std::make_shared<SystemTimeSource>()),
      executor_(std::make_unique<Executor>(options.executor_threads)),
      topics_(std::make_unique<TopicRegistry>(*executor_)),
      services_(std::make_unique<ServiceRegistry>()),
      timers_(std::make_unique<TimerManager>(*time_source_, *executor_)),
      parameters_(std::make_unique<ParameterManager>(std::move(options.parameters))) {
  if (options.network.listen || options.network.port != 0 || !options.network.peers.empty()) {
    peer_network_ = std::make_unique<net::PeerNetwork>(*this, std::move(options.network));
  }
  if (options.publish_parameter_events) {
    parameters_->set_event_callback([this](const ParameterEvent& event) {
      topics_->publish("parameter_events", event);
    });
  }
}

Context::~Context() { shutdown(); }

void Context::register_node(const std::string& name) {
  std::lock_guard<std::mutex> lock(node_mutex_);
  if (!node_names_.insert(name).second) {
    throw std::runtime_error("노드 이름이 겹친다: '" + name + "' (같은 클래스를 여럿 만들면 name을 따로 줘라)");
  }
}

std::vector<std::string> Context::node_names() const {
  std::lock_guard<std::mutex> lock(node_mutex_);
  return {node_names_.begin(), node_names_.end()};
}

void Context::shutdown() {
  if (shutdown_) {
    return;
  }
  shutdown_ = true;
  if (peer_network_) {
    peer_network_->shutdown();
  }
  // 콜백이 더 이상 새 일을 만들지 못하게 한 뒤, 큐에 남은 실행을 기다린다.
  timers_->shutdown();
  services_->shutdown();
  topics_->shutdown();
  executor_->join();
  executor_->shutdown();
}

}  // namespace krosc
