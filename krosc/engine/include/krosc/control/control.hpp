#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// control의 control_interface. hardware_interface라는 ROS2 이름 대신 역할 기준으로 둔다.
//   Control: 하드웨어 하나(로봇 팔, 보드)의 control 인터페이스.
//   StateInterface/CommandInterface: joint1/position 같은 이름의 값 하나.
namespace krosc::control {

class StateInterface {
 public:
  explicit StateInterface(std::string name, double value = 0.0)
      : name_(std::move(name)), value_(value) {}

  [[nodiscard]] const std::string& name() const { return name_; }
  [[nodiscard]] double get() const { return value_; }
  void set(double value) { value_ = value; }

 private:
  std::string name_;
  double value_ = 0.0;
};

class CommandInterface {
 public:
  explicit CommandInterface(std::string name, double value = 0.0)
      : name_(std::move(name)), value_(value) {}

  [[nodiscard]] const std::string& name() const { return name_; }
  [[nodiscard]] double get() const { return value_; }
  void set(double value) { value_ = value; }

 private:
  std::string name_;
  double value_ = 0.0;
};

struct ControlInfo {
  std::string name = "hardware";
  std::string type;
  std::string group;
};

// control 역할 어댑터. 물리 장치 공통 lifecycle은 krosc/hardware/hardware.hpp의 Hardware다.
// on_init -> on_activate -> read/write 반복 -> on_deactivate.
class Control {
 public:
  virtual ~Control() = default;

  virtual bool on_init(const ControlInfo& info) {
    info_ = info;
    return true;
  }
  virtual bool on_activate() { return true; }
  virtual bool on_deactivate() { return true; }

  [[nodiscard]] virtual std::vector<std::shared_ptr<StateInterface>> export_state_interfaces() = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<CommandInterface>> export_command_interfaces() = 0;

  virtual bool read(double time, double period) = 0;
  virtual bool write(double time, double period) = 0;

  [[nodiscard]] const ControlInfo& info() const { return info_; }

 private:
  ControlInfo info_;
};

using ControlFactory = std::function<std::unique_ptr<Control>()>;

class ControlRegistry {
 public:
  void add(const std::string& type, ControlFactory factory) { factories_[type] = std::move(factory); }
  [[nodiscard]] std::unique_ptr<Control> create(const std::string& type) const {
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
  std::map<std::string, ControlFactory> factories_;
};

}  // namespace krosc::control
