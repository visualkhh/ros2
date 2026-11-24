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
  std::lock_guard<std::mutex> lock(state->mutex);
  auto it = state->services.find(service->name);
  if (it != state->services.end() && it->second == service) {
    state->services.erase(it);
  }
  service_.reset();
}

ServiceRegistry::ServiceRegistry() : state_(std::make_shared<detail::ServiceState>()) {}
ServiceRegistry::~ServiceRegistry() { shutdown(); }

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
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->stopping = true;
  state_->services.clear();
}

}  // namespace krosc
