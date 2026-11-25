#pragma once

// ParamValue(로컬 variant) <-> rcl_interfaces::ParameterValue(와이어) 변환.

#include <cstdint>

#include "krosc/msgs/rcl_interfaces.hpp"
#include "krosc/parameter.hpp"

namespace krosc {

[[nodiscard]] std::uint8_t parameter_type_id(const ParamValue& value);
[[nodiscard]] msgs::rcl_interfaces::ParameterValue parameter_value_message(const ParamValue& value);
[[nodiscard]] ParamValue param_value_from_message(
    const msgs::rcl_interfaces::ParameterValue& value);

}  // namespace krosc
