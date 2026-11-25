#pragma once

// rcl_interfaces의 파라미터 서비스 메시지 축소판.
// ROS 2와 같은 서비스 이름/의미를 쓴다:
//   <node>/get_parameters, <node>/list_parameters, <node>/describe_parameters,
//   <node>/set_parameters, <node>/set_parameters_atomically

#include <cstdint>
#include <string>
#include <vector>

namespace krosc::msgs::rcl_interfaces {

enum class ParameterType : std::uint8_t {
  NotSet = 0,
  Bool = 1,
  Integer = 2,
  Double = 3,
  String = 4,
  BoolArray = 6,
  IntegerArray = 7,
  DoubleArray = 8,
  StringArray = 9,
};

struct ParameterValue {
  std::uint8_t type = static_cast<std::uint8_t>(ParameterType::NotSet);
  bool bool_value = false;
  std::int64_t integer_value = 0;
  double double_value = 0.0;
  std::string string_value;
  std::vector<bool> bool_array_value;
  std::vector<std::int64_t> integer_array_value;
  std::vector<double> double_array_value;
  std::vector<std::string> string_array_value;
};

struct Parameter {
  std::string name;
  ParameterValue value;
};

struct SetParametersResult {
  bool successful = false;
  std::string reason;
};

struct GetParametersRequest {
  std::vector<std::string> names;
};
struct GetParametersResponse {
  std::vector<ParameterValue> values;
};

struct SetParametersRequest {
  std::vector<Parameter> parameters;
};
struct SetParametersResponse {
  std::vector<SetParametersResult> results;
};

struct ListParametersRequest {
  std::vector<std::string> prefixes;
  std::uint64_t depth = 0;
};
struct ListParametersResponse {
  std::vector<std::string> result;
};

struct ParameterDescriptor {
  std::string name;
  std::uint8_t type = static_cast<std::uint8_t>(ParameterType::NotSet);
  std::string description;
  std::string additional_info;
  bool read_only = false;
};

struct DescribeParametersRequest {
  std::vector<std::string> names;
};
struct DescribeParametersResponse {
  std::vector<ParameterDescriptor> descriptors;
};

}  // namespace krosc::msgs::rcl_interfaces
