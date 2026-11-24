#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "krosc/node.hpp"
#include "krosc/ros2_control/hardware_interface.hpp"

// ros2_control의 controller_interface 흉내.
//   unconfigured -> inactive -> active 수명 주기를 가진다.
//   컨트롤러는 command interface를 읽고/쓰며, update()가 controller_manager 주기마다 불린다.
namespace krosc::ros2_control {

struct ControllerContext {
  Node* node = nullptr;
  std::string name;
  std::map<std::string, std::shared_ptr<StateInterface>> state_interfaces;
  std::map<std::string, std::shared_ptr<CommandInterface>> command_interfaces;

  [[nodiscard]] std::shared_ptr<StateInterface> state(const std::string& interface_name) const {
    const auto it = state_interfaces.find(interface_name);
    return it == state_interfaces.end() ? nullptr : it->second;
  }
  [[nodiscard]] std::shared_ptr<CommandInterface> command(const std::string& interface_name) const {
    const auto it = command_interfaces.find(interface_name);
    return it == command_interfaces.end() ? nullptr : it->second;
  }
};

class ControllerInterface {
 public:
  virtual ~ControllerInterface() = default;

  virtual bool on_init(ControllerContext& context) {
    context_ = &context;
    return true;
  }
  virtual bool on_configure() { return true; }
  virtual bool on_activate() { return true; }
  virtual bool on_deactivate() { return true; }
  virtual bool on_cleanup() { return true; }

  virtual void update(double time, double period) = 0;

  // 이 컨트롤러가 쓰는 command interface 이름. controller_manager가 활성화 충돌을 막는다.
  [[nodiscard]] virtual std::vector<std::string> command_interface_names() const { return {}; }

  [[nodiscard]] const std::string& name() const { return context_ == nullptr ? empty_name() : context_->name; }
  [[nodiscard]] virtual std::string type() const = 0;

 protected:
  [[nodiscard]] ControllerContext& context() const { return *context_; }
  ControllerContext* context_ = nullptr;

 private:
  static const std::string& empty_name() {
    static const std::string value;
    return value;
  }
};

using ControllerFactory = std::function<std::unique_ptr<ControllerInterface>()>;

class ControllerRegistry {
 public:
  void add(const std::string& type, ControllerFactory factory) { factories_[type] = std::move(factory); }
  [[nodiscard]] std::unique_ptr<ControllerInterface> create(const std::string& type) const {
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
  std::map<std::string, ControllerFactory> factories_;
};

// JSB, JTC, forward command controller를 등록해 돌려준다.
ControllerRegistry builtin_controllers();

}  // namespace krosc::ros2_control
