#include "krosc/krosc.hpp"

#include <cstdio>
#include <utility>

namespace krosc {

Krosc::Krosc() : Krosc(nullptr, Context::Options{}) {}

namespace {

Context::Options network_options(std::vector<std::string> peers, std::uint16_t port) {
  Context::Options options;
  options.network.peers = std::move(peers);
  options.network.port = port;
  options.network.listen = port != 0;
  return options;
}

Context::Options network_options(net::PeerNetworkOptions network) {
  Context::Options options;
  options.network = std::move(network);
  return options;
}

}  // namespace

Krosc::Krosc(NodeCreate node_create, Context::Options options) : context_(std::make_unique<Context>(std::move(options))) {
  if (!node_create) {
    return;
  }
  try {
    nodes_ = node_create(*context_);
  } catch (...) {
    context_->shutdown();
    throw;
  }
  for (const auto& unused : context_->parameters().unused_overrides()) {
    std::fprintf(stderr, "[krosc] 값을 줬는데 선언된 파라미터가 없다 (오타?): %s\n", unused.c_str());
  }
}

Krosc::Krosc(NodeCreate node_create, std::vector<std::string> peers, std::uint16_t port)
    : Krosc(std::move(node_create), network_options(std::move(peers), port)) {}

Krosc::Krosc(NodeCreate node_create, net::PeerNetworkOptions network)
    : Krosc(std::move(node_create), network_options(std::move(network))) {}

Krosc::~Krosc() { shutdown(); }

void Krosc::shutdown() {
  if (shutdown_) {
    return;
  }
  shutdown_ = true;
  if (context_) {
    context_->shutdown();
  }
  nodes_.clear();
}

}  // namespace krosc
