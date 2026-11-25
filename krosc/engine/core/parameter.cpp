#include "krosc/parameter.hpp"

#include "krosc/msgs/rcl_interfaces.hpp"
#include "krosc/parameter_rpc.hpp"

#include <algorithm>
#include <utility>

namespace krosc {

namespace {

template <class T>
std::string vector_to_string(const std::vector<T>& values) {
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      out << ", ";
    }
    out << values[i];
  }
  out << ']';
  return out.str();
}

template <>
std::string vector_to_string<bool>(const std::vector<bool>& values) {
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      out << ", ";
    }
    out << (values[i] ? "true" : "false");
  }
  out << ']';
  return out.str();
}

}  // namespace

std::string param_type_name(const ParamValue& value) {
  switch (value.index()) {
    case 0:
      return "bool";
    case 1:
      return "int64";
    case 2:
      return "double";
    case 3:
      return "string";
    case 4:
      return "bool[]";
    case 5:
      return "int64[]";
    case 6:
      return "double[]";
    case 7:
      return "string[]";
    default:
      return "unknown";
  }
}

std::string param_to_string(const ParamValue& value) {
  return std::visit(
      [](const auto& item) -> std::string {
        using T = std::decay_t<decltype(item)>;
        if constexpr (std::is_same_v<T, bool>) {
          return item ? "true" : "false";
        } else if constexpr (std::is_same_v<T, std::int64_t>) {
          return std::to_string(item);
        } else if constexpr (std::is_same_v<T, double>) {
          std::ostringstream out;
          out << item;
          return out.str();
        } else if constexpr (std::is_same_v<T, std::string>) {
          return "'" + item + "'";
        } else {
          return vector_to_string(item);
        }
      },
      value);
}

ParameterManager::ParameterManager(Overrides overrides) : overrides_(std::move(overrides)) {}

void ParameterManager::set_event_callback(std::function<void(const ParameterEvent&)> callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  on_event_ = std::move(callback);
}

void ParameterManager::set_node_overrides(const std::string& node, std::map<std::string, ParamValue> values) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& target = node_overrides_[node];
  for (auto& [name, value] : values) {
    target[name] = std::move(value);
  }
}

ParameterManager::Entry ParameterManager::find_entry(const std::string& node, const std::string& name) const {
  auto node_it = entries_.find(node);
  if (node_it != entries_.end()) {
    auto value_it = node_it->second.find(name);
    if (value_it != node_it->second.end()) {
      return value_it->second;
    }
  }
  throw ParameterNotDeclaredError(node + "." + name + "는 선언되지 않았다");
}

ParamValue ParameterManager::get(const std::string& node, const std::string& name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return find_entry(node, name).value;
}

void ParameterManager::set(const std::string& node, const std::string& name, const ParamValue& value) {
  std::function<void(const ParamValue&)> on_change;
  std::optional<ParamValue> old;
  bool unchanged = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto node_it = entries_.find(node);
    if (node_it == entries_.end()) {
      throw ParameterNotDeclaredError(node + "." + name + "는 선언되지 않았다");
    }
    auto value_it = node_it->second.find(name);
    if (value_it == node_it->second.end()) {
      throw ParameterNotDeclaredError(node + "." + name + "는 선언되지 않았다");
    }
    Entry& entry = value_it->second;
    if (entry.value.index() != value.index()) {
      throw ParameterTypeError(node + "." + name + "는 " + param_type_name(entry.value) + " 타입으로 선언됐다: " +
                               param_to_string(value) + " (" + param_type_name(value) + ")");
    }
    if (entry.validator && !entry.validator(value)) {
      throw ParameterRejectedError(node + "." + name + "가 값 " + param_to_string(value) + "를 거절했다");
    }
    if (entry.value == value) {
      on_change = entry.on_change;  // Python kros처럼 같은 값이어도 on_change는 부른다 (이벤트는 내지 않는다)
      unchanged = true;
    } else {
      old = entry.value;
      entry.value = value;
      on_change = entry.on_change;
    }
  }
  if (on_change) {
    on_change(value);
  }
  if (!unchanged) {
    publish_event(ParameterEvent{node, name, false, old, value});
  }
}

std::map<std::string, ParamValue> ParameterManager::declared(const std::string& node) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, ParamValue> result;
  auto it = entries_.find(node);
  if (it != entries_.end()) {
    for (const auto& [name, entry] : it->second) {
      result[name] = entry.value;
    }
  }
  return result;
}

std::vector<std::string> ParameterManager::unused_overrides() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> unused;
  auto collect = [&](const Overrides& source) {
    for (const auto& [node, values] : source) {
      auto node_it = entries_.find(node);
      for (const auto& [name, value] : values) {
        (void)value;
        if (node_it == entries_.end() || node_it->second.count(name) == 0) {
          unused.push_back(node + "." + name);
        }
      }
    }
  };
  collect(overrides_);
  collect(node_overrides_);
  std::sort(unused.begin(), unused.end());
  unused.erase(std::unique(unused.begin(), unused.end()), unused.end());
  return unused;
}

void ParameterManager::publish_event(const ParameterEvent& event) const {
  std::function<void(const ParameterEvent&)> callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    callback = on_event_;
  }
  if (callback) {
    callback(event);
  }
}

}  // namespace krosc

namespace krosc {

namespace rcl = msgs::rcl_interfaces;

std::uint8_t parameter_type_id(const ParamValue& value) {
  switch (value.index()) {
    case 0:
      return static_cast<std::uint8_t>(rcl::ParameterType::Bool);
    case 1:
      return static_cast<std::uint8_t>(rcl::ParameterType::Integer);
    case 2:
      return static_cast<std::uint8_t>(rcl::ParameterType::Double);
    case 3:
      return static_cast<std::uint8_t>(rcl::ParameterType::String);
    case 4:
      return static_cast<std::uint8_t>(rcl::ParameterType::BoolArray);
    case 5:
      return static_cast<std::uint8_t>(rcl::ParameterType::IntegerArray);
    case 6:
      return static_cast<std::uint8_t>(rcl::ParameterType::DoubleArray);
    case 7:
      return static_cast<std::uint8_t>(rcl::ParameterType::StringArray);
    default:
      return static_cast<std::uint8_t>(rcl::ParameterType::NotSet);
  }
}

msgs::rcl_interfaces::ParameterValue parameter_value_message(const ParamValue& value) {
  msgs::rcl_interfaces::ParameterValue message;
  message.type = parameter_type_id(value);
  switch (value.index()) {
    case 0:
      message.bool_value = std::get<bool>(value);
      break;
    case 1:
      message.integer_value = std::get<std::int64_t>(value);
      break;
    case 2:
      message.double_value = std::get<double>(value);
      break;
    case 3:
      message.string_value = std::get<std::string>(value);
      break;
    case 4:
      for (const bool item : std::get<std::vector<bool>>(value)) {
        message.bool_array_value.push_back(item);
      }
      break;
    case 5:
      message.integer_array_value = std::get<std::vector<std::int64_t>>(value);
      break;
    case 6:
      message.double_array_value = std::get<std::vector<double>>(value);
      break;
    case 7:
      message.string_array_value = std::get<std::vector<std::string>>(value);
      break;
    default:
      break;
  }
  return message;
}

ParamValue param_value_from_message(const msgs::rcl_interfaces::ParameterValue& value) {
  switch (static_cast<rcl::ParameterType>(value.type)) {
    case rcl::ParameterType::Bool:
      return value.bool_value;
    case rcl::ParameterType::Integer:
      return value.integer_value;
    case rcl::ParameterType::Double:
      return value.double_value;
    case rcl::ParameterType::String:
      return value.string_value;
    case rcl::ParameterType::BoolArray:
      return value.bool_array_value;
    case rcl::ParameterType::IntegerArray:
      return value.integer_array_value;
    case rcl::ParameterType::DoubleArray:
      return value.double_array_value;
    case rcl::ParameterType::StringArray:
      return value.string_array_value;
    case rcl::ParameterType::NotSet:
    default:
      throw ParameterTypeError("ParameterValue type이 설정되지 않았다");
  }
}

}  // namespace krosc
