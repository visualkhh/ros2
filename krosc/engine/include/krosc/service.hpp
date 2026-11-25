#pragma once

#include <any>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>
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

using ServiceResponseEncoder =
    std::function<std::vector<std::uint8_t>(std::type_index, const void*)>;

struct ServiceBase {
  std::string name;
  std::type_index request_type{typeid(void)};
  std::type_index response_type{typeid(void)};
  std::string type_name;
  virtual ~ServiceBase() = default;
  virtual std::any invoke(const std::any& request) = 0;
  // RPC 응답을 std::any를 거치지 않고 바로 인코딩한다.
  virtual std::vector<std::uint8_t> invoke_encoded(const std::any& request,
                                                   const ServiceResponseEncoder& encoder) = 0;
};

using RemoteServiceCall = std::function<std::any(
    const std::string& name, std::type_index request_type, const void* request,
    std::type_index response_type)>;

struct ServiceState {
  mutable std::mutex mutex;
  std::unordered_map<std::string, std::shared_ptr<ServiceBase>> services;
  RemoteServiceCall remote_call;
  std::function<void()> change_hook;
  bool stopping = false;
};

template <class Req, class Res>
struct ServiceImpl final : ServiceBase {
  std::function<Res(const Req&)> handler;
  std::any invoke(const std::any& request) override { return handler(std::any_cast<const Req&>(request)); }
  std::vector<std::uint8_t> invoke_encoded(const std::any& request,
                                           const ServiceResponseEncoder& encoder) override {
    const Res response = handler(std::any_cast<const Req&>(request));
    return encoder(std::type_index(typeid(Res)), &response);
  }
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

// 서비스 이름 -> 서버 하나. 로컬 서버가 있으면 직접(동기) 실행하고,
// 없으면 PeerNetwork가 설정한 remote_call로 원격 피어에 포워딩한다.
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
    notify_change(state);
    return ServiceServer(state, service);
  }

  template <class Req, class Res>
  Res call(const std::string& name, const Req& request) {
    auto state = state_;
    std::shared_ptr<detail::ServiceBase> service;
    detail::RemoteServiceCall remote;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      auto it = state->services.find(name);
      if (it == state->services.end()) {
        remote = state->remote_call;
      } else {
        service = it->second;
      }
    }
    if (!service) {
      if (remote) {
        std::any response = remote(name, std::type_index(typeid(Req)), &request,
                                   std::type_index(typeid(Res)));
        if (!response.has_value()) {
          throw ServiceNotFoundError("no server for service '" + name + "'");
        }
        if (response.type() != typeid(Res)) {
          throw ServiceTypeError("service '" + name + "' returned " + response.type().name());
        }
        return std::any_cast<Res>(std::move(response));
      }
      throw ServiceNotFoundError("no server for service '" + name + "'");
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
  [[nodiscard]] std::optional<std::pair<std::type_index, std::type_index>> service_types(
      const std::string& name) const;
  // 네트워크에서 받은 요청을 로컬 서비스로 실행한다. 없거나 타입이 다르면 빈 any.
  [[nodiscard]] std::any invoke_any(const std::string& name, const std::any& request) const;
  // 응답을 바로 인코딩한다. 서비스가 없거나 요청 타입이 다르면 빈 벡터.
  [[nodiscard]] std::vector<std::uint8_t> invoke_encoded(
      const std::string& name, const std::any& request,
      const detail::ServiceResponseEncoder& encoder) const;

  // 로컬에 서비스가 없을 때 원격 피어로 넘긴다 (PeerNetwork가 설정).
  void set_remote_call(detail::RemoteServiceCall hook);
  // 서비스 생성/삭제 시 discovery 갱신용.
  void set_change_hook(std::function<void()> hook);

  void shutdown();

 private:
  static void notify_change(const std::shared_ptr<detail::ServiceState>& state);

  std::shared_ptr<detail::ServiceState> state_;
};

}  // namespace krosc
