#pragma once

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "krosc/manipulation/arm_chain.hpp"
#include "krosc/node.hpp"

// MoveIt의 PlannerManager 플러그인 흉내.
//   move_group은 MotionPlanRequest(여기서는 PlanningRequest)를 플러그인에 넘기고 궤적을 받는다.
namespace krosc::moveit {

struct PlanningRequest {
  std::array<double, 4> start{};
  std::array<double, 4> goal{};
  bool cartesian = true;
  double max_joint_speed = 0.35;
  double max_joint_accel = 1.0;
  double sample_time = 0.05;
  double cartesian_speed = 0.08;
  double cartesian_accel = 0.25;
};

struct PlanningResult {
  bool success = false;
  manipulation::Trajectory trajectory;
  std::string error;
};

class MoveItPlannerPlugin {
 public:
  virtual ~MoveItPlannerPlugin() = default;

  virtual bool initialize(Node& node, const std::string& name, const manipulation::ArmChain& chain,
                          const manipulation::PlanOptions& options) {
    (void)name;
    node_ = &node;
    chain_ = &chain;
    options_ = options;
    return true;
  }

  [[nodiscard]] virtual PlanningResult plan(const PlanningRequest& request) = 0;
  [[nodiscard]] virtual std::string type() const = 0;

 protected:
  [[nodiscard]] PlanningResult run(const PlanningRequest& request, bool cartesian) const;

  Node* node_ = nullptr;
  const manipulation::ArmChain* chain_ = nullptr;
  manipulation::PlanOptions options_;
};

using PlannerFactory = std::function<std::unique_ptr<MoveItPlannerPlugin>()>;

class PlannerRegistry {
 public:
  void add(const std::string& type, PlannerFactory factory) { factories_[type] = std::move(factory); }
  [[nodiscard]] std::unique_ptr<MoveItPlannerPlugin> create(const std::string& type) const {
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
  std::map<std::string, PlannerFactory> factories_;
};

PlannerRegistry builtin_planners();

}  // namespace krosc::moveit
