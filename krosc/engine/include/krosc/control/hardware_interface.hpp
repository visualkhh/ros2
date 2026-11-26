#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// control의 hardware_interface 흉내.
//   SystemInterface: 하드웨어 하나(로봇 팔, 보드)의 read/write.
//   StateInterface/CommandInterface: joint1/position 같은 이름의 값 하나.
// 실제 ROS2는 pluginlib로 hardware plugin을 dlopen하지만, krosc는 정적 레지스트리로 같은 구조를 만든다.
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

struct HardwareInfo {
  std::string name = "hardware";
  std::string type;
  std::string group;
};

// control의 SystemInterface. on_init -> on_activate -> read/write 반복 -> on_deactivate.
class SystemInterface {
 public:
  virtual ~SystemInterface() = default;

  virtual bool on_init(const HardwareInfo& info) {
    info_ = info;
    return true;
  }
  virtual bool on_activate() { return true; }
  virtual bool on_deactivate() { return true; }

  [[nodiscard]] virtual std::vector<std::shared_ptr<StateInterface>> export_state_interfaces() = 0;
  [[nodiscard]] virtual std::vector<std::shared_ptr<CommandInterface>> export_command_interfaces() = 0;

  virtual bool read(double time, double period) = 0;
  virtual bool write(double time, double period) = 0;

  [[nodiscard]] const HardwareInfo& info() const { return info_; }

 private:
  HardwareInfo info_;
};

using SystemFactory = std::function<std::unique_ptr<SystemInterface>()>;

class HardwareRegistry {
 public:
  void add(const std::string& type, SystemFactory factory) { factories_[type] = std::move(factory); }
  [[nodiscard]] std::unique_ptr<SystemInterface> create(const std::string& type) const {
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
  std::map<std::string, SystemFactory> factories_;
};

}  // namespace krosc::control
