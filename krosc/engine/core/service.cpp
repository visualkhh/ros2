#include "krosc/service.hpp"

#include <algorithm>
#include <vector>

namespace krosc {

void ServiceServer::destroy() {
  auto service = service_.lock();
  auto state = state_.lock();
  if (!service || !state) {
    return;
  }
  std::function<void()> hook;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    auto it = state->services.find(service->name);
    if (it != state->services.end() && it->second == service) {
      state->services.erase(it);
    }
    hook = state->change_hook;
  }
  service_.reset();
  if (hook) {
    hook();
  }
}

ServiceRegistry::ServiceRegistry() : state_(std::make_shared<detail::ServiceState>()) {}
ServiceRegistry::~ServiceRegistry() { shutdown(); }

void ServiceRegistry::notify_change(const std::shared_ptr<detail::ServiceState>& state) {
  std::function<void()> hook;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    hook = state->change_hook;
  }
  if (hook) {
    hook();
  }
}

void ServiceRegistry::set_remote_call(detail::RemoteServiceCall hook) {
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->remote_call = std::move(hook);
}

void ServiceRegistry::set_change_hook(std::function<void()> hook) {
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->change_hook = std::move(hook);
}

std::optional<std::pair<std::type_index, std::type_index>> ServiceRegistry::service_types(
    const std::string& name) const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  const auto found = state_->services.find(name);
  if (found == state_->services.end()) {
    return std::nullopt;
  }
  return std::make_pair(found->second->request_type, found->second->response_type);
}

std::any ServiceRegistry::invoke_any(const std::string& name, const std::any& request) const {
  std::shared_ptr<detail::ServiceBase> service;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    const auto found = state_->services.find(name);
    if (found == state_->services.end()) {
      return {};
    }
    service = found->second;
  }
  if (service->request_type != std::type_index(request.type())) {
    return {};
  }
  return service->invoke(request);
}

std::vector<std::uint8_t> ServiceRegistry::invoke_encoded(
    const std::string& name, const std::any& request,
    const detail::ServiceResponseEncoder& encoder) const {
  std::shared_ptr<detail::ServiceBase> service;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    const auto found = state_->services.find(name);
    if (found == state_->services.end()) {
      return {};
    }
    service = found->second;
  }
  if (service->request_type != std::type_index(request.type())) {
    return {};
  }
  return service->invoke_encoded(request, encoder);
}

bool ServiceRegistry::has_service(const std::string& name) const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->services.count(name) != 0;
}

std::vector<std::string> ServiceRegistry::names() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  std::vector<std::string> result;
  result.reserve(state_->services.size());
  for (const auto& [name, service] : state_->services) {
    (void)service;
    result.push_back(name);
  }
  std::sort(result.begin(), result.end());
  return result;
}

void ServiceRegistry::shutdown() {
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->stopping = true;
    state_->services.clear();
    state_->remote_call = {};
  }
  notify_change(state_);
}

}  // namespace krosc
