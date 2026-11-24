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

class PlannerManager {
 public:
  virtual ~PlannerManager() = default;

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
  Node* node_ = nullptr;
  const manipulation::ArmChain* chain_ = nullptr;
  manipulation::PlanOptions options_;
};

using PlannerFactory = std::function<std::unique_ptr<PlannerManager>()>;

class PlannerRegistry {
 public:
  void add(const std::string& type, PlannerFactory factory) { factories_[type] = std::move(factory); }
  [[nodiscard]] std::unique_ptr<PlannerManager> create(const std::string& type) const {
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

class CartesianPlanner final : public PlannerManager {
 public:
  [[nodiscard]] PlanningResult plan(const PlanningRequest& request) override;
  [[nodiscard]] std::string type() const override { return "krosc_moveit/CartesianPlanner"; }
};

class JointSpacePlanner final : public PlannerManager {
 public:
  [[nodiscard]] PlanningResult plan(const PlanningRequest& request) override;
  [[nodiscard]] std::string type() const override { return "krosc_moveit/JointSpacePlanner"; }
};

inline PlanningResult run_planner(const manipulation::ArmChain& chain,
                                  const manipulation::PlanOptions& base_options,
                                  const PlanningRequest& request, bool cartesian) {
  PlanningResult result;
  manipulation::PlanOptions options = base_options;
  options.max_joint_speed = request.max_joint_speed;
  options.max_joint_accel = request.max_joint_accel;
  options.sample_time = request.sample_time;
  options.cartesian_speed = request.cartesian_speed;
  options.cartesian_accel = request.cartesian_accel;
  options.cartesian = cartesian;
  auto trajectory = manipulation::plan_trajectory(chain, request.start, request.goal, options,
                                                  result.error);
  if (!trajectory) {
    return result;
  }
  result.success = true;
  result.trajectory = std::move(*trajectory);
  return result;
}

inline PlanningResult CartesianPlanner::plan(const PlanningRequest& request) {
  if (chain_ == nullptr) {
    return PlanningResult{false, {}, "URDF 체인이 없다"};
  }
  return run_planner(*chain_, options_, request, request.cartesian);
}

inline PlanningResult JointSpacePlanner::plan(const PlanningRequest& request) {
  if (chain_ == nullptr) {
    return PlanningResult{false, {}, "URDF 체인이 없다"};
  }
  return run_planner(*chain_, options_, request, false);
}

inline PlannerRegistry builtin_planners() {
  PlannerRegistry registry;
  registry.add("krosc_moveit/CartesianPlanner", [] { return std::make_unique<CartesianPlanner>(); });
  registry.add("krosc_moveit/JointSpacePlanner", [] { return std::make_unique<JointSpacePlanner>(); });
  return registry;
}

}  // namespace krosc::moveit
