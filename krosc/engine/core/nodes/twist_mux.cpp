#include "krosc/nodes/twist_mux.hpp"

#include <algorithm>
#include <utility>

namespace krosc::nodes {

TwistMux::TwistMux(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      cmd_vel_out_(declare_parameter<std::string>("cmd_vel_out", "cmd_vel_out")),
      topics_(declare_parameter<std::vector<std::string>>("topics", {"navigation", "teleop"})),
      locks_(declare_parameter<std::vector<std::string>>("locks", std::vector<std::string>{})) {
  started_ = now();
  const std::map<std::string, std::tuple<std::string, double, std::int64_t>> defaults = {
      {"navigation", {"cmd_vel_nav", 0.5, 10}},
      {"teleop", {"cmd_vel_teleop", 0.5, 100}},
  };
  for (const auto& raw_name : topics_.value()) {
    const std::string input_name = raw_name;
    std::string default_topic = input_name;
    double default_timeout = 0.5;
    std::int64_t default_priority = 50;
    if (auto it = defaults.find(input_name); it != defaults.end()) {
      default_topic = std::get<0>(it->second);
      default_timeout = std::get<1>(it->second);
      default_priority = std::get<2>(it->second);
    }
    const auto topic = declare_parameter<std::string>(input_name + ".topic", default_topic).value();
    const double timeout =
        declare_parameter<double>(input_name + ".timeout", default_timeout, [](double value) { return value > 0.0; })
            .value();
    const std::int64_t priority =
        declare_parameter<std::int64_t>(input_name + ".priority", default_priority,
                                        [](std::int64_t value) { return value >= 0 && value <= 255; })
            .value();
    Input input;
    input.topic = topic;
    input.timeout = timeout;
    input.priority = static_cast<int>(priority);
    inputs_[input_name] = input;
    subscriptions_.push_back(subscribe<msgs::Twist>(
        topic, [this, input_name](const msgs::Twist& twist) { on_twist(input_name, twist); }, QOS_COMMAND));
  }
  for (const auto& raw_name : locks_.value()) {
    const std::string lock_name = raw_name;
    const auto topic = declare_parameter<std::string>(lock_name + ".topic", lock_name).value();
    const double timeout =
        declare_parameter<double>(lock_name + ".timeout", 0.0, [](double value) { return value >= 0.0; }).value();
    const std::int64_t priority =
        declare_parameter<std::int64_t>(lock_name + ".priority", 255,
                                        [](std::int64_t value) { return value >= 0 && value <= 255; })
            .value();
    Lock lock;
    lock.topic = topic;
    lock.timeout = timeout;
    lock.priority = static_cast<int>(priority);
    lock.last = started_;
    lock_inputs_[lock_name] = lock;
    lock_subscriptions_.push_back(subscribe<msgs::Bool>(
        topic, [this, lock_name](const msgs::Bool& flag) { on_lock(lock_name, flag); }, QOS_COMMAND));
  }
}

int TwistMux::lock_priority() const {
  const double current = now();
  int level = -1;
  for (const auto& [name, lock] : lock_inputs_) {
    (void)name;
    if (lock.state || (lock.timeout > 0.0 && current - lock.last > lock.timeout)) {
      level = std::max(level, lock.priority);
    }
  }
  return level;
}

std::string TwistMux::active() const {
  const double current = now();
  const int blocked = lock_priority();
  std::optional<std::pair<int, std::string>> winner;
  for (const auto& [name, input] : inputs_) {
    if (!input.received || current - input.last > input.timeout || input.priority <= blocked) {
      continue;
    }
    const std::pair<int, std::string> candidate{input.priority, name};
    if (!winner || candidate > *winner) {
      winner = candidate;
    }
  }
  return winner ? winner->second : std::string{};
}

void TwistMux::on_twist(const std::string& name, const msgs::Twist& twist) {
  const double current = now();
  auto it = inputs_.find(name);
  if (it == inputs_.end()) {
    return;
  }
  it->second.received = true;
  it->second.last = current;
  if (active() == name && !cmd_vel_out_.value().empty()) {
    publish(cmd_vel_out_.value(), twist);
  }
}

void TwistMux::on_lock(const std::string& name, const msgs::Bool& flag) {
  auto it = lock_inputs_.find(name);
  if (it == lock_inputs_.end()) {
    return;
  }
  it->second.state = flag.data;
  it->second.last = now();
}

}  // namespace krosc::nodes
