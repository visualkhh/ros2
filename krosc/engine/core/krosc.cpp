#include "krosc/krosc.hpp"

#include <cstdio>
#include <utility>

namespace krosc {

Krosc::Krosc() : Krosc(nullptr, Context::Options{}) {}

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
