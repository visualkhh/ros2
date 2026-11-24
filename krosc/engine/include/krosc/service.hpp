#pragma once

#include <any>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"

namespace krosc {

class ServiceNotFoundError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
class ServiceTypeError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

namespace detail {

struct ServiceBase {
  std::string name;
  std::type_index request_type{typeid(void)};
  std::type_index response_type{typeid(void)};
  std::string type_name;
  virtual ~ServiceBase() = default;
  virtual std::any invoke(const std::any& request) = 0;
};

struct ServiceState {
  mutable std::mutex mutex;
  std::unordered_map<std::string, std::shared_ptr<ServiceBase>> services;
  bool stopping = false;
};

template <class Req, class Res>
struct ServiceImpl final : ServiceBase {
  std::function<Res(const Req&)> handler;
  std::any invoke(const std::any& request) override { return handler(std::any_cast<const Req&>(request)); }
};

}  // namespace detail

class ServiceRegistry;

// 서비스 서버 손잡이. destroy()로 내린다. Node가 만든 서버는 Node가 소멸할 때 정리한다.
class ServiceServer {
 public:
  ServiceServer() = default;

  void destroy();
  [[nodiscard]] bool valid() const { return !service_.expired(); }
  [[nodiscard]] std::string name() const {
    auto service = service_.lock();
    return service ? service->name : std::string{};
  }

 private:
  friend class ServiceRegistry;
  ServiceServer(std::weak_ptr<detail::ServiceState> state, std::shared_ptr<detail::ServiceBase> service)
      : state_(std::move(state)), service_(std::move(service)) {}

  std::weak_ptr<detail::ServiceState> state_;
  std::weak_ptr<detail::ServiceBase> service_;
};

// 서비스 이름 -> 서버 하나 (한 프로세스 안). 호출은 지금은 직접(동기) 실행한다.
// 핸들러 예외는 호출자에게 그대로 전파된다.
class ServiceRegistry {
 public:
  ServiceRegistry();
  ~ServiceRegistry();

  ServiceRegistry(const ServiceRegistry&) = delete;
  ServiceRegistry& operator=(const ServiceRegistry&) = delete;

  template <class Req, class Res>
  ServiceServer create_service(const std::string& name, std::function<Res(const Req&)> handler) {
    auto state = state_;
    auto service = std::make_shared<detail::ServiceImpl<Req, Res>>();
    service->name = name;
    service->request_type = std::type_index(typeid(Req));
    service->response_type = std::type_index(typeid(Res));
    service->type_name = std::string(msgs::message_type_name<Req>()) + " -> " + std::string(msgs::message_type_name<Res>());
    service->handler = std::move(handler);
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->stopping) {
        throw std::runtime_error("ServiceRegistry가 종료됐다");
      }
      if (state->services.count(name) != 0) {
        throw std::runtime_error("service '" + name + "' already has a server");
      }
      state->services.emplace(name, service);
    }
    return ServiceServer(state, service);
  }

  template <class Req, class Res>
  Res call(const std::string& name, const Req& request) {
    auto state = state_;
    std::shared_ptr<detail::ServiceBase> service;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      auto it = state->services.find(name);
      if (it == state->services.end()) {
        throw ServiceNotFoundError("no server for service '" + name + "'");
      }
      service = it->second;
    }
    if (service->request_type != std::type_index(typeid(Req)) ||
        service->response_type != std::type_index(typeid(Res))) {
      throw ServiceTypeError("service '" + name + "' is " + service->type_name + ", got " +
                             std::string(msgs::message_type_name<Req>()) + " -> " +
                             std::string(msgs::message_type_name<Res>()));
    }
    std::any response = service->invoke(std::any(request));
    if (response.type() != typeid(Res)) {
      throw ServiceTypeError("service '" + name + "' returned " + response.type().name());
    }
    return std::any_cast<Res>(std::move(response));
  }

  [[nodiscard]] bool has_service(const std::string& name) const;
  [[nodiscard]] std::vector<std::string> names() const;
  void shutdown();

 private:
  std::shared_ptr<detail::ServiceState> state_;
};

}  // namespace krosc
