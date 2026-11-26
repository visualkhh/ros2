#include "krosc/control/controller_manager.hpp"

#include <algorithm>
#include <exception>
#include <utility>

#include "krosc/control/diff_drive_controller.hpp"
#include "krosc/control/forward_command_controller.hpp"
#include "krosc/control/joint_state_broadcaster.hpp"
#include "krosc/control/joint_trajectory_controller.hpp"

namespace krosc::control {

namespace {

std::string state_name(ControllerManager::State state) {
  switch (state) {
    case ControllerManager::State::Unconfigured:
      return "unconfigured";
    case ControllerManager::State::Inactive:
      return "inactive";
    case ControllerManager::State::Active:
      return "active";
    case ControllerManager::State::Finalized:
      return "finalized";
  }
  return "unknown";
}

}  // namespace

ControllerManager::ControllerManager(Context& context, std::string name,
                                     std::map<std::string, ParamValue> parameters,
                                     const HardwareRegistry& hardware_registry,
                                     ControllerRegistry controllers)
    : Node(context, std::move(name), std::move(parameters)),
      update_rate_(declare_parameter<double>("update_rate", 50.0,
                                             [](double value) { return value > 0.0; })),
      hardware_plugin_(declare_parameter<std::string>("hardware.plugin", "")),
      hardware_name_(declare_parameter<std::string>("hardware.name", "controller_manager_hardware")),
      controller_plugins_(declare_parameter<std::vector<std::string>>("controller_plugins",
                                                                      std::vector<std::string>{})),
      registry_(std::move(controllers)) {
  if (hardware_plugin_.value().empty()) {
    logger().error("controller_manager: hardware.plugin이 비어 있다");
    throw std::runtime_error("controller_manager: hardware.plugin is required");
  }
  hardware_ = hardware_registry.create(hardware_plugin_.value());
  if (!hardware_) {
    throw std::runtime_error("controller_manager: hardware plugin not found: " +
                             hardware_plugin_.value());
  }
  const HardwareInfo info{hardware_name_.value(), hardware_plugin_.value(), "krosc"};
  if (!hardware_->on_init(info)) {
    throw std::runtime_error("controller_manager: hardware on_init failed");
  }
  if (!hardware_->on_activate()) {
    throw std::runtime_error("controller_manager: hardware on_activate failed");
  }
  hardware_active_ = true;
  for (const auto& state : hardware_->export_state_interfaces()) {
    state_interfaces_[state->name()] = state;
  }
  for (const auto& command : hardware_->export_command_interfaces()) {
    command_interfaces_[command->name()] = command;
  }
  // 컨트롤러를 활성화하기 전에 한 번 읽어 실제 상태를 채운다 (JTC가 0을 hold하지 않게).
  (void)hardware_->read(now(), 1.0 / update_rate_.value());

  for (const auto& controller_name : controller_plugins_.value()) {
    if (!load_controller(controller_name)) {
      throw std::runtime_error("controller_manager: failed to load controller '" + controller_name +
                               "'");
    }
    auto* entry = find(controller_name);
    const bool auto_activate =
        declare_parameter<bool>(controller_name + ".auto_activate", true).value();
    if (entry != nullptr) {
      entry->auto_activate = auto_activate;
    }
    if (!configure_controller(controller_name)) {
      throw std::runtime_error("controller_manager: failed to configure controller '" +
                               controller_name + "'");
    }
  }
  std::vector<std::string> start;
  for (const auto& entry : entries_) {
    if (entry->auto_activate) {
      start.push_back(entry->name);
    }
  }
  if (!start.empty() && !switch_controller(start, {}, 1)) {
    logger().warn("controller_manager: 일부 컨트롤러를 활성화하지 못했다");
  }

  list_service_ = create_service<ListControllersRequest, ListControllersResponse>(
      this->name() + "/list_controllers",
      [this](const ListControllersRequest&) {
        ListControllersResponse response;
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& entry : entries_) {
          response.name.push_back(entry->name);
          response.type.push_back(entry->type);
          response.state.push_back(state_name(entry->state));
        }
        return response;
      });
  load_service_ = create_service<LoadControllerRequest, LoadControllerResponse>(
      this->name() + "/load_controller", [this](const LoadControllerRequest& request) {
        LoadControllerResponse response;
        response.ok = load_controller(request.name);
        response.message = response.ok ? "loaded" : "load failed";
        return response;
      });
  configure_service_ = create_service<ConfigureControllerRequest, ConfigureControllerResponse>(
      this->name() + "/configure_controller",
      [this](const ConfigureControllerRequest& request) {
        ConfigureControllerResponse response;
        response.ok = configure_controller(request.name);
        response.message = response.ok ? "configured" : "configure failed";
        return response;
      });
  switch_service_ = create_service<SwitchControllerRequest, SwitchControllerResponse>(
      this->name() + "/switch_controller", [this](const SwitchControllerRequest& request) {
        SwitchControllerResponse response;
        response.ok = switch_controller(request.start_controllers, request.stop_controllers,
                                        request.strictness);
        response.message = response.ok ? "switched" : "switch failed";
        return response;
      });
  update_timer_ = create_timer(1.0 / update_rate_.value(), [this] { update(); });
  logger().info("controller_manager: hardware=%s interfaces=%zu/%zu controllers=%zu rate=%.0fHz",
                hardware_plugin_.value().c_str(), state_interfaces_.size(),
                command_interfaces_.size(), entries_.size(), update_rate_.value());
}

ControllerManager::~ControllerManager() {
  update_timer_.cancel();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& entry : entries_) {
      if (entry->state == State::Active) {
        entry->controller->on_deactivate();
        entry->state = State::Inactive;
      }
      entry->controller->on_cleanup();
      entry->state = State::Finalized;
    }
  }
  if (hardware_ && hardware_active_) {
    hardware_->on_deactivate();
    hardware_active_ = false;
  }
}

ControllerManager::Entry* ControllerManager::find(const std::string& controller_name) {
  for (auto& entry : entries_) {
    if (entry->name == controller_name) {
      return entry.get();
    }
  }
  return nullptr;
}

const ControllerManager::Entry* ControllerManager::find(const std::string& controller_name) const {
  for (const auto& entry : entries_) {
    if (entry->name == controller_name) {
      return entry.get();
    }
  }
  return nullptr;
}

bool ControllerManager::load_controller(const std::string& controller_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (find(controller_name) != nullptr) {
    return true;
  }
  std::string type;
  try {
    type = get_parameter_as<std::string>(controller_name + ".type");
  } catch (const std::exception&) {
    type = declare_parameter<std::string>(controller_name + ".type", "").value();
  }
  if (type.empty() || !registry_.contains(type)) {
    logger().error("controller_manager: controller plugin not found: %s", type.c_str());
    return false;
  }
  auto entry = std::make_unique<Entry>();
  entry->name = controller_name;
  entry->type = type;
  entry->controller = registry_.create(type);
  if (!entry->controller) {
    return false;
  }
  entry->context = std::make_unique<ControllerContext>();
  entry->context->node = this;
  entry->context->name = controller_name;
  entry->context->state_interfaces = state_interfaces_;
  entry->context->command_interfaces = command_interfaces_;
  if (!entry->controller->on_init(*entry->context)) {
    logger().error("controller_manager: %s on_init 실패", controller_name.c_str());
    return false;
  }
  entry->state = State::Unconfigured;
  entries_.push_back(std::move(entry));
  return true;
}

bool ControllerManager::configure_controller(const std::string& controller_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  Entry* entry = find(controller_name);
  if (entry == nullptr) {
    return false;
  }
  if (entry->state == State::Inactive || entry->state == State::Active) {
    return true;
  }
  if (entry->state != State::Unconfigured) {
    return false;
  }
  if (!entry->controller->on_configure()) {
    return false;
  }
  entry->state = State::Inactive;
  return true;
}

std::vector<std::string> ControllerManager::active_claims_locked(
    const std::string& except) const {
  std::vector<std::string> claims;
  for (const auto& entry : entries_) {
    if (entry->state != State::Active || entry->name == except) {
      continue;
    }
    const auto names = entry->controller->command_interface_names();
    claims.insert(claims.end(), names.begin(), names.end());
  }
  return claims;
}

bool ControllerManager::claimed_by_other_locked(
    const std::string& controller_name, const std::vector<std::string>& interfaces) const {
  const auto claims = active_claims_locked(controller_name);
  for (const auto& interface : interfaces) {
    if (std::find(claims.begin(), claims.end(), interface) != claims.end()) {
      return true;
    }
  }
  return false;
}

bool ControllerManager::activate_locked(Entry& entry) {
  if (entry.state == State::Active) {
    return true;
  }
  if (entry.state != State::Inactive) {
    return false;
  }
  if (claimed_by_other_locked(entry.name, entry.controller->command_interface_names())) {
    logger().error("controller_manager: %s가 쓰는 command interface를 다른 컨트롤러가 이미 쓰고 있다",
                   entry.name.c_str());
    return false;
  }
  if (!entry.controller->on_activate()) {
    return false;
  }
  entry.state = State::Active;
  return true;
}

void ControllerManager::deactivate_locked(Entry& entry) {
  if (entry.state != State::Active) {
    return;
  }
  entry.controller->on_deactivate();
  entry.state = State::Inactive;
}

bool ControllerManager::switch_controller(const std::vector<std::string>& start,
                                          const std::vector<std::string>& stop,
                                          std::int64_t strictness) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& name : stop) {
    Entry* entry = find(name);
    if (entry == nullptr) {
      if (strictness != 0) {
        logger().error("controller_manager: stop할 컨트롤러가 없다: %s", name.c_str());
        return false;
      }
      continue;
    }
    deactivate_locked(*entry);
  }
  bool ok = true;
  for (const auto& name : start) {
    Entry* entry = find(name);
    if (entry == nullptr) {
      logger().error("controller_manager: start할 컨트롤러가 없다: %s", name.c_str());
      ok = false;
      if (strictness != 0) {
        return false;
      }
      continue;
    }
    if (!activate_locked(*entry)) {
      ok = false;
      if (strictness != 0) {
        return false;
      }
    }
  }
  return ok;
}

ControllerManager::State ControllerManager::state_of(const std::string& controller_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const Entry* entry = find(controller_name);
  return entry == nullptr ? State::Finalized : entry->state;
}

std::vector<std::string> ControllerManager::controller_names() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> names;
  names.reserve(entries_.size());
  for (const auto& entry : entries_) {
    names.push_back(entry->name);
  }
  return names;
}

std::string ControllerManager::state_text(State state) const { return state_name(state); }

void ControllerManager::update() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!hardware_active_) {
    return;
  }
  const double time = now();
  const double period = 1.0 / update_rate_.value();
  hardware_->read(time, period);
  for (auto& entry : entries_) {
    if (entry->state == State::Active) {
      entry->controller->update(time, period);
    }
  }
  hardware_->write(time, period);
}

ControllerRegistry builtin_controllers() {
  ControllerRegistry registry;
  registry.add("joint_state_broadcaster/JointStateBroadcaster",
               [] { return std::make_unique<JointStateBroadcaster>(); });
  registry.add("joint_trajectory_controller/JointTrajectoryController",
               [] { return std::make_unique<JointTrajectoryController>(); });
  registry.add("forward_command_controller/ForwardCommandController",
               [] { return std::make_unique<ForwardCommandController>(); });
  registry.add("diff_drive_controller/DiffDriveController",
               [] { return std::make_unique<DiffDriveController>(); });
  return registry;
}

}  // namespace krosc::control
