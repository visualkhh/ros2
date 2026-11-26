#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "krosc/node.hpp"
#include "krosc/control/controller.hpp"
#include "krosc/control/control.hpp"
#include "krosc/time/timer.hpp"

namespace krosc::control {

// controller_manager 서비스 타입 (controller_manager_msgs의 축소판).
struct ListControllersRequest {};
struct ListControllersResponse {
  std::vector<std::string> name;
  std::vector<std::string> type;
  std::vector<std::string> state;
};

struct LoadControllerRequest {
  std::string name;
};
struct LoadControllerResponse {
  bool ok = false;
  std::string message;
};

struct ConfigureControllerRequest {
  std::string name;
};
struct ConfigureControllerResponse {
  bool ok = false;
  std::string message;
};

struct SwitchControllerRequest {
  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;
  std::int64_t strictness = 0;  // 0 = best effort, 1 = strict
  bool start_asap = false;
  double timeout = 0.0;
};
struct SwitchControllerResponse {
  bool ok = false;
  std::string message;
};

// control controller_manager 흉내.
//   hardware(plugin) -> state/command interfaces -> controller(plugin) -> update loop.
//   서비스: list_controllers / load_controller / configure_controller / switch_controller.
//   실제 ROS2와 같이 command interface 하나를 두 컨트롤러가 동시에 쓸 수 없다.
class ControllerManager final : public Node {
 public:
  enum class State { Unconfigured, Inactive, Active, Finalized };

  explicit ControllerManager(Context& context, std::string name = "controller_manager",
                             std::map<std::string, ParamValue> parameters = {},
                             const ControlRegistry& hardware_registry = {},
                             ControllerRegistry controllers = builtin_controllers());
  // 외부(HardwareManager)가 하드웨어 lifecycle/read/write를 소유하는 모드.
  // 이 모드에서는 타이머를 만들지 않고 update(time, period)만 제공한다.
  ControllerManager(Context& context, std::string name,
                    std::map<std::string, ParamValue> parameters,
                    Control& hardware,
                    ControllerRegistry controllers = builtin_controllers());
  ~ControllerManager() override;

  ControllerManager(const ControllerManager&) = delete;
  ControllerManager& operator=(const ControllerManager&) = delete;

  // 외부 모드에서 HardwareManager의 control tick이 호출한다.
  void update(double time, double period);

  [[nodiscard]] bool load_controller(const std::string& controller_name);
  [[nodiscard]] bool configure_controller(const std::string& controller_name);
  [[nodiscard]] bool switch_controller(const std::vector<std::string>& start,
                                       const std::vector<std::string>& stop,
                                       std::int64_t strictness = 1);
  [[nodiscard]] State state_of(const std::string& controller_name) const;
  [[nodiscard]] std::vector<std::string> controller_names() const;
  [[nodiscard]] std::string state_text(State state) const;

  [[nodiscard]] const std::map<std::string, std::shared_ptr<StateInterface>>& state_interfaces() const {
    return state_interfaces_;
  }
  [[nodiscard]] const std::map<std::string, std::shared_ptr<CommandInterface>>& command_interfaces() const {
    return command_interfaces_;
  }

 private:
  struct Entry {
    std::string name;
    std::string type;
    std::unique_ptr<Controller> controller;
    std::unique_ptr<ControllerContext> context;
    State state = State::Unconfigured;
    bool auto_activate = true;
  };

  void update();
  void configure_controllers();
  [[nodiscard]] Entry* find(const std::string& controller_name);
  [[nodiscard]] const Entry* find(const std::string& controller_name) const;
  [[nodiscard]] bool activate_locked(Entry& entry);
  void deactivate_locked(Entry& entry);
  [[nodiscard]] bool claimed_by_other_locked(const std::string& controller_name,
                                             const std::vector<std::string>& interfaces) const;
  [[nodiscard]] std::vector<std::string> active_claims_locked(const std::string& except) const;

  Parameter<double> update_rate_;
  Parameter<std::string> hardware_plugin_;
  Parameter<std::string> hardware_name_;
  Parameter<std::vector<std::string>> controller_plugins_;

  ControllerRegistry registry_;
  std::unique_ptr<Control> owned_hardware_;
  Control* hardware_ = nullptr;
  bool external_hardware_ = false;
  bool hardware_active_ = false;
  std::map<std::string, std::shared_ptr<StateInterface>> state_interfaces_;
  std::map<std::string, std::shared_ptr<CommandInterface>> command_interfaces_;

  mutable std::mutex mutex_;
  std::vector<std::unique_ptr<Entry>> entries_;
  std::string hardware_status_ = "unconfigured";
  Timer update_timer_;
  ServiceServer list_service_;
  ServiceServer load_service_;
  ServiceServer configure_service_;
  ServiceServer switch_service_;
};

}  // namespace krosc::control
