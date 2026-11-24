#include "krosc/node.hpp"

#include <atomic>
#include <stdexcept>
#include <utility>

namespace krosc {

namespace {
std::atomic<std::uint64_t> next_node_id{1};
}

Node::Node(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : context_(context), name_(name.empty() ? "node_" + std::to_string(next_node_id.fetch_add(1)) : std::move(name)) {
  context_.register_node(name_);
  if (!parameters.empty()) {
    context_.parameters().set_node_overrides(name_, std::move(parameters));
  }
  log_level_ = context_.parameters().declare<std::string>(
      name_, "log_level", "INFO",
      [](const std::string& level) { return parse_log_level(level).has_value(); });
  logger_ = std::make_unique<NodeLogger>(
      name_, [this](const msgs::Log& log) { publish(ROSOUT_TOPIC_NAME, log); },
      [this] { return context_.now(); },
      [this] {
        const auto level = parse_log_level(log_level_.value());
        return level.value_or(LogLevel::Info);
      });
}

Node::~Node() { shutdown(); }

void Node::shutdown() {
  for (auto& timer : timers_) {
    timer.cancel();
  }
  timers_.clear();
  for (auto& subscription : subscriptions_) {
    subscription->cancel();
  }
  subscriptions_.clear();
  for (auto& service : services_) {
    service.destroy();
  }
  services_.clear();
}

}  // namespace krosc
