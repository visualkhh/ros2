#pragma once

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "krosc/manipulation/arm_chain.hpp"
#include "krosc/node.hpp"

// MoveIt의 kinematics_base 플러그인 흉내.
//   move_group은 URDF 체인을 만들고, IK/FK는 이 플러그인에 맡긴다.
namespace krosc::moveit {

struct KinematicsQuery {
  msgs::Vector3 target;
  double pitch_down = 0.0;
  std::array<double, 4> seed{};
  bool fast = false;
};

struct KinematicsResult {
  bool success = false;
  std::array<double, 4> solution{};
  std::string error;
};

class KinematicsBase {
 public:
  virtual ~KinematicsBase() = default;

  virtual bool initialize(Node& node, const std::string& name, const manipulation::ArmChain& chain) {
    (void)name;
    node_ = &node;
    chain_ = &chain;
    return true;
  }

  [[nodiscard]] virtual KinematicsResult search_position_ik(const KinematicsQuery& query) const = 0;
  [[nodiscard]] virtual msgs::Vector3 get_position_fk(const std::array<double, 4>& joints) const = 0;

 protected:
  Node* node_ = nullptr;
  const manipulation::ArmChain* chain_ = nullptr;
};

using KinematicsFactory = std::function<std::unique_ptr<KinematicsBase>()>;

class KinematicsRegistry {
 public:
  void add(const std::string& type, KinematicsFactory factory) { factories_[type] = std::move(factory); }
  [[nodiscard]] std::unique_ptr<KinematicsBase> create(const std::string& type) const {
    const auto it = factories_.find(type);
    return it == factories_.end() ? nullptr : it->second();
  }
  [[nodiscard]] bool contains(const std::string& type) const { return factories_.count(type) != 0; }
  [[nodiscard]] std::vector<std::string> names() const {
    std::vector<std::string> result;
    for (const auto& [type, factory] : factories_) {
      (void)factory;
      result.push_back(type);
    }
    return result;
  }

 private:
  std::map<std::string, KinematicsFactory> factories_;
};

// krosc의 URDF 체인 해석 IK를 KinematicsBase 플러그인으로 감싼다.
class ArmChainKinematics final : public KinematicsBase {
 public:
  [[nodiscard]] KinematicsResult search_position_ik(const KinematicsQuery& query) const override {
    KinematicsResult result;
    if (chain_ == nullptr || !chain_->valid()) {
      result.error = "URDF 체인이 없다";
      return result;
    }
    result.success = chain_->ik(query.target, query.pitch_down, query.seed, result.solution,
                               result.error, query.fast);
    return result;
  }

  [[nodiscard]] msgs::Vector3 get_position_fk(const std::array<double, 4>& joints) const override {
    return chain_ == nullptr ? msgs::Vector3{} : chain_->tcp(joints);
  }
};

inline KinematicsRegistry builtin_kinematics() {
  KinematicsRegistry registry;
  registry.add("krosc_moveit/ArmChainKinematics", [] { return std::make_unique<ArmChainKinematics>(); });
  return registry;
}

}  // namespace krosc::moveit
