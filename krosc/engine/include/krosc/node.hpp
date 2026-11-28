#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/log/log.hpp"
#include "krosc/msgs.hpp"
#include "krosc/parameter.hpp"
#include "krosc/qos.hpp"
#include "krosc/service.hpp"
#include "krosc/time/timer.hpp"
#include "krosc/topic.hpp"

namespace krosc {

// ROS 2 callback group. 기본은 MutuallyExclusive: 한 노드의 구독/타이머 콜백은 서로 동시에 돌지 않아
// 노드 멤버에 락이 필요 없다 (노드끼리는 병렬). Reentrant는 노드 줄을 거치지 않고 바로 Executor로 간다.
// 서비스 핸들러는 지금도 호출한 쪽 스레드에서 돈다 (동기 호출 데드락을 피하려고 줄에 넣지 않는다).
enum class CallbackGroup { MutuallyExclusive, Reentrant };

// krosc 노드의 기본형: Context를 반드시 가진다. 이름은 Context 안에서 유일해야 한다.
// 파라미터를 선언해서 가지며, 실행 중에 publish/subscribe/service/timer를 만든다.
class Node {
 public:
  Node(Context& context, std::string name = {}, std::map<std::string, ParamValue> parameters = {});
  virtual ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  [[nodiscard]] const std::string& name() const { return name_; }
  [[nodiscard]] Context& context() { return context_; }
  [[nodiscard]] const Context& context() const { return context_; }
  [[nodiscard]] double now() const { return context_.now(); }
  [[nodiscard]] NodeLogger& logger() { return *logger_; }

  // 이 노드가 만든 타이머/구독/서비스를 정리한다 (여러 번 불러도 안전).
  void shutdown();

  template <class T>
  Subscription<T> subscribe(const std::string& topic, std::function<void(const T&)> callback,
                            QoS qos = QOS_DEFAULT, CallbackGroup group = CallbackGroup::MutuallyExclusive) {
    auto subscription = context_.topics().subscribe<T>(topic, std::move(callback), qos, strand_for(group));
    // 타입 소거해 두었다가 Node가 소멸할 때 정리한다. (TopicRegistry도 자체적으로 구독을 보관한다)
    subscriptions_.push_back(subscription.base());
    return subscription;
  }

  // 같은 토픽을 자주 발행하면 손잡이를 만들어 두고 쓴다 (publish마다 이름 조회가 없다).
  template <class T>
  Publisher<T> create_publisher(const std::string& topic, QoS qos = QOS_DEFAULT) {
    return context_.topics().create_publisher<T>(topic, qos);
  }

  template <class T>
  void publish(const std::string& topic, T message, QoS qos = QOS_DEFAULT) {
    context_.topics().publish<T>(topic, std::move(message), qos);
  }

  template <class T>
  void publish(const std::string& topic, std::shared_ptr<const T> message, QoS qos = QOS_DEFAULT) {
    context_.topics().publish<T>(topic, std::move(message), qos);
  }

  Timer create_timer(double period, std::function<void()> callback,
                     CallbackGroup group = CallbackGroup::MutuallyExclusive) {
    Timer timer = context_.timers().create(period, std::move(callback), name_, strand_for(group));
    timers_.push_back(timer);
    return timer;
  }

  template <class Req, class Res>
  ServiceServer create_service(const std::string& service_name, std::function<Res(const Req&)> handler) {
    ServiceServer server = context_.services().create_service<Req, Res>(service_name, std::move(handler));
    services_.push_back(server);
    return server;
  }

  template <class Req, class Res>
  Res call_service(const std::string& service_name, const Req& request) {
    return context_.services().call<Req, Res>(service_name, request);
  }

  template <class T>
  Parameter<T> declare_parameter(const std::string& parameter_name, T default_value,
                                 std::function<bool(const T&)> validator = {},
                                 std::function<void(const T&)> on_change = {}) {
    return context_.parameters().declare<T>(name_, parameter_name, std::move(default_value), std::move(validator),
                                            std::move(on_change));
  }

  Parameter<std::string> declare_parameter(const std::string& parameter_name, const char* default_value,
                                           std::function<bool(const std::string&)> validator = {},
                                           std::function<void(const std::string&)> on_change = {}) {
    return context_.parameters().declare<std::string>(
        name_, parameter_name, std::string(default_value == nullptr ? "" : default_value), std::move(validator),
        std::move(on_change));
  }

  Parameter<std::int64_t> declare_parameter(const std::string& parameter_name, int default_value,
                                            std::function<bool(const std::int64_t&)> validator = {},
                                            std::function<void(const std::int64_t&)> on_change = {}) {
    return declare_parameter<std::int64_t>(parameter_name, static_cast<std::int64_t>(default_value),
                                           std::move(validator), std::move(on_change));
  }

  [[nodiscard]] ParamValue get_parameter(const std::string& parameter_name) const {
    return context_.parameters().get(name_, parameter_name);
  }

  template <class T>
  [[nodiscard]] T get_parameter_as(const std::string& parameter_name) const {
    return context_.parameters().get_as<T>(name_, parameter_name);
  }

  void set_parameter(const std::string& parameter_name, const ParamValue& value) {
    context_.parameters().set(name_, parameter_name, value);
  }

  template <class T>
  void set_parameter(const std::string& parameter_name, const T& value) {
    context_.parameters().set<T>(name_, parameter_name, value);
  }

  void set_parameter(const std::string& parameter_name, int value) {
    context_.parameters().set<std::int64_t>(name_, parameter_name, static_cast<std::int64_t>(value));
  }

 private:
  void create_parameter_services();
  [[nodiscard]] std::shared_ptr<Strand> strand_for(CallbackGroup group) const {
    return group == CallbackGroup::MutuallyExclusive ? strand_ : nullptr;
  }

  Context& context_;
  std::shared_ptr<Strand> strand_ = std::make_shared<Strand>(context_.executor());
  std::string name_;
  Parameter<std::string> log_level_;
  std::unique_ptr<NodeLogger> logger_;
  std::vector<std::shared_ptr<detail::SubscriptionBase>> subscriptions_;
  std::vector<Timer> timers_;
  std::vector<ServiceServer> services_;
};

}  // namespace krosc
