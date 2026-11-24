#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "krosc/msgs.hpp"

namespace krosc {

// 파이썬 kros의 ParameterValue와 같은 값 종류: bool/int/double/string 또는 그 벡터.
using ParamValue =
    std::variant<bool, std::int64_t, double, std::string, std::vector<bool>, std::vector<std::int64_t>,
                 std::vector<double>, std::vector<std::string>>;

[[nodiscard]] std::string param_type_name(const ParamValue& value);
[[nodiscard]] std::string param_to_string(const ParamValue& value);

class ParameterNotDeclaredError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
class ParameterAlreadyDeclaredError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
class ParameterRejectedError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
class ParameterTypeError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct ParameterEvent {
  std::string node;
  std::string name;
  bool declared = true;  // true면 선언, false면 변경
  std::optional<ParamValue> old_value;
  ParamValue new_value;
};

class ParameterManager;

// 선언한 파라미터 하나의 손잡이. value()는 항상 현재 값이다.
template <class T>
class Parameter {
 public:
  Parameter() = default;
  Parameter(ParameterManager* manager, std::string node, std::string name)
      : manager_(manager), node_(std::move(node)), name_(std::move(name)) {}

  [[nodiscard]] T value() const;
  operator T() const { return value(); }  // NOLINT(google-explicit-constructor)
  void set(const T& value);

  [[nodiscard]] const std::string& node() const { return node_; }
  [[nodiscard]] const std::string& name() const { return name_; }

 private:
  ParameterManager* manager_ = nullptr;
  std::string node_;
  std::string name_;
};

class ParameterManager {
 public:
  using Overrides = std::map<std::string, std::map<std::string, ParamValue>>;

  explicit ParameterManager(Overrides overrides = {});

  void set_event_callback(std::function<void(const ParameterEvent&)> callback);
  void set_node_overrides(const std::string& node, std::map<std::string, ParamValue> values);

  template <class T>
  Parameter<T> declare(const std::string& node, const std::string& name, T default_value,
                       std::function<bool(const T&)> validator = {},
                       std::function<void(const T&)> on_change = {}) {
    ParamValue default_param{std::move(default_value)};
    std::function<bool(const ParamValue&)> erased_validator;
    if (validator) {
      erased_validator = [validator = std::move(validator)](const ParamValue& value) {
        return validator(std::get<T>(value));
      };
    }
    std::function<void(const ParamValue&)> erased_change;
    if (on_change) {
      erased_change = [on_change = std::move(on_change)](const ParamValue& value) { on_change(std::get<T>(value)); };
    }

    ParamValue declared_value = default_param;
    std::optional<ParamValue> chosen;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (entries_[node].count(name) != 0) {
        throw ParameterAlreadyDeclaredError(node + "." + name + "는 이미 선언됐다");
      }
      if (auto node_it = node_overrides_.find(node); node_it != node_overrides_.end()) {
        if (auto it = node_it->second.find(name); it != node_it->second.end()) {
          chosen = it->second;
        }
      }
      if (auto it = overrides_.find(node); it != overrides_.end()) {
        if (auto value_it = it->second.find(name); value_it != it->second.end()) {
          chosen = value_it->second;  // 전역(Kros) 값이 노드별 값보다 세다 (Python kros와 같다)
        }
      }
    }
    if (chosen) {
      if (chosen->index() != default_param.index()) {
        throw ParameterTypeError(node + "." + name + "는 " + param_type_name(default_param) + " 타입이다: " +
                                 param_to_string(*chosen) + " (" + param_type_name(*chosen) + ")");
      }
      declared_value = *chosen;
    }

    Entry entry;
    entry.value = declared_value;
    entry.validator = std::move(erased_validator);
    entry.on_change = std::move(erased_change);

    if (entry.validator && !entry.validator(declared_value)) {
      throw ParameterRejectedError(node + "." + name + "가 값 " + param_to_string(declared_value) + "를 거절했다");
    }
    // 기본값 자체가 검증을 못 통과하면 선언이 잘못된 것이다 (덮어쓴 값이 유효해도).
    if (chosen && entry.validator && !entry.validator(default_param)) {
      throw ParameterRejectedError(node + "." + name + "의 기본값 " + param_to_string(default_param) +
                                   "가 검증을 통과하지 못한다");
    }
    if (declared_value.index() != default_param.index()) {
      throw ParameterTypeError(node + "." + name + "의 타입이 기본값과 다르다");
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto& node_entries = entries_[node];
      if (node_entries.count(name) != 0) {
        throw ParameterAlreadyDeclaredError(node + "." + name + "는 이미 선언됐다");
      }
      node_entries[name] = std::move(entry);
    }
    publish_event(ParameterEvent{node, name, true, std::nullopt, declared_value});
    return Parameter<T>(this, node, name);
  }

  [[nodiscard]] ParamValue get(const std::string& node, const std::string& name) const;

  template <class T>
  [[nodiscard]] T get_as(const std::string& node, const std::string& name) const {
    const ParamValue value = get(node, name);
    if (!std::holds_alternative<T>(value)) {
      throw ParameterTypeError(node + "." + name + "는 " + param_type_name(value) + " 타입이다");
    }
    return std::get<T>(value);
  }

  void set(const std::string& node, const std::string& name, const ParamValue& value);

  template <class T>
  void set(const std::string& node, const std::string& name, const T& value) {
    set(node, name, ParamValue{value});
  }

  [[nodiscard]] std::map<std::string, ParamValue> declared(const std::string& node) const;
  [[nodiscard]] std::vector<std::string> unused_overrides() const;

 private:
  struct Entry {
    ParamValue value;
    std::function<bool(const ParamValue&)> validator;
    std::function<void(const ParamValue&)> on_change;
  };

  void publish_event(const ParameterEvent& event) const;
  [[nodiscard]] Entry find_entry(const std::string& node, const std::string& name) const;

  mutable std::mutex mutex_;
  Overrides overrides_;
  Overrides node_overrides_;
  std::map<std::string, std::map<std::string, Entry>> entries_;
  std::function<void(const ParameterEvent&)> on_event_;
};

template <class T>
T Parameter<T>::value() const {
  if (manager_ == nullptr) {
    throw ParameterNotDeclaredError("초기화되지 않은 Parameter 손잡이");
  }
  return manager_->get_as<T>(node_, name_);
}

template <class T>
void Parameter<T>::set(const T& value) {
  if (manager_ == nullptr) {
    throw ParameterNotDeclaredError("초기화되지 않은 Parameter 손잡이");
  }
  manager_->set<T>(node_, name_, value);
}

}  // namespace krosc

namespace krosc::msgs {
template <>
constexpr std::string_view message_type_name<krosc::ParameterEvent>() {
  return "rcl_interfaces/ParameterEvent";
}
}  // namespace krosc::msgs
