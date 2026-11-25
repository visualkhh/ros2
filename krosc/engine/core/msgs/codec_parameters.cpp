#include "krosc/net/rpc_codec.hpp"

#include "krosc/msgs/rcl_interfaces.hpp"

#include "codec_util.hpp"

namespace krosc::msgs::codec {

void register_parameter_rpc_codecs() {
  net::register_rpc_codec<rcl_interfaces::GetParametersRequest>(
      "rcl_interfaces/srv/GetParameters_Request",
      [](net::RpcWriter& w, const rcl_interfaces::GetParametersRequest& value) {
        encode_strings(w, value.names);
      },
      [](net::RpcReader& r, rcl_interfaces::GetParametersRequest& value) {
        value.names = decode_strings(r);
        return true;
      });
  net::register_rpc_codec<rcl_interfaces::GetParametersResponse>(
      "rcl_interfaces/srv/GetParameters_Response",
      [](net::RpcWriter& w, const rcl_interfaces::GetParametersResponse& value) {
        w.u32(static_cast<std::uint32_t>(value.values.size()));
        for (const auto& item : value.values) {
          encode_parameter_value(w, item);
        }
      },
      [](net::RpcReader& r, rcl_interfaces::GetParametersResponse& value) {
        const std::uint32_t count = r.u32();
        if (count > 1000000) {
          return false;
        }
        value.values.reserve(count);
        for (std::uint32_t i = 0; i < count; ++i) {
          value.values.push_back(decode_parameter_value(r));
        }
        return true;
      });
  net::register_rpc_codec<rcl_interfaces::SetParametersRequest>(
      "rcl_interfaces/srv/SetParameters_Request",
      [](net::RpcWriter& w, const rcl_interfaces::SetParametersRequest& value) {
        w.u32(static_cast<std::uint32_t>(value.parameters.size()));
        for (const auto& parameter : value.parameters) {
          w.str(parameter.name);
          encode_parameter_value(w, parameter.value);
        }
      },
      [](net::RpcReader& r, rcl_interfaces::SetParametersRequest& value) {
        const std::uint32_t count = r.u32();
        if (count > 1000000) {
          return false;
        }
        value.parameters.reserve(count);
        for (std::uint32_t i = 0; i < count; ++i) {
          rcl_interfaces::Parameter parameter;
          parameter.name = r.str();
          parameter.value = decode_parameter_value(r);
          value.parameters.push_back(std::move(parameter));
        }
        return true;
      });
  net::register_rpc_codec<rcl_interfaces::SetParametersResponse>(
      "rcl_interfaces/srv/SetParameters_Response",
      [](net::RpcWriter& w, const rcl_interfaces::SetParametersResponse& value) {
        w.u32(static_cast<std::uint32_t>(value.results.size()));
        for (const auto& item : value.results) {
          w.u8(item.successful ? 1U : 0U);
          w.str(item.reason);
        }
      },
      [](net::RpcReader& r, rcl_interfaces::SetParametersResponse& value) {
        const std::uint32_t count = r.u32();
        if (count > 1000000) {
          return false;
        }
        value.results.reserve(count);
        for (std::uint32_t i = 0; i < count; ++i) {
          rcl_interfaces::SetParametersResult item;
          item.successful = r.u8() != 0;
          item.reason = r.str();
          value.results.push_back(std::move(item));
        }
        return true;
      });
  net::register_rpc_codec<rcl_interfaces::ListParametersRequest>(
      "rcl_interfaces/srv/ListParameters_Request",
      [](net::RpcWriter& w, const rcl_interfaces::ListParametersRequest& value) {
        encode_strings(w, value.prefixes);
        w.u64(value.depth);
      },
      [](net::RpcReader& r, rcl_interfaces::ListParametersRequest& value) {
        value.prefixes = decode_strings(r);
        value.depth = r.u64();
        return true;
      });
  net::register_rpc_codec<rcl_interfaces::ListParametersResponse>(
      "rcl_interfaces/srv/ListParameters_Response",
      [](net::RpcWriter& w, const rcl_interfaces::ListParametersResponse& value) {
        encode_strings(w, value.result);
      },
      [](net::RpcReader& r, rcl_interfaces::ListParametersResponse& value) {
        value.result = decode_strings(r);
        return true;
      });
  net::register_rpc_codec<rcl_interfaces::DescribeParametersRequest>(
      "rcl_interfaces/srv/DescribeParameters_Request",
      [](net::RpcWriter& w, const rcl_interfaces::DescribeParametersRequest& value) {
        encode_strings(w, value.names);
      },
      [](net::RpcReader& r, rcl_interfaces::DescribeParametersRequest& value) {
        value.names = decode_strings(r);
        return true;
      });
  net::register_rpc_codec<rcl_interfaces::DescribeParametersResponse>(
      "rcl_interfaces/srv/DescribeParameters_Response",
      [](net::RpcWriter& w, const rcl_interfaces::DescribeParametersResponse& value) {
        w.u32(static_cast<std::uint32_t>(value.descriptors.size()));
        for (const auto& descriptor : value.descriptors) {
          w.str(descriptor.name);
          w.u8(descriptor.type);
          w.str(descriptor.description);
          w.str(descriptor.additional_info);
          w.u8(descriptor.read_only ? 1U : 0U);
        }
      },
      [](net::RpcReader& r, rcl_interfaces::DescribeParametersResponse& value) {
        const std::uint32_t count = r.u32();
        if (count > 1000000) {
          return false;
        }
        value.descriptors.reserve(count);
        for (std::uint32_t i = 0; i < count; ++i) {
          rcl_interfaces::ParameterDescriptor descriptor;
          descriptor.name = r.str();
          descriptor.type = r.u8();
          descriptor.description = r.str();
          descriptor.additional_info = r.str();
          descriptor.read_only = r.u8() != 0;
          value.descriptors.push_back(std::move(descriptor));
        }
        return true;
      });
}

}  // namespace krosc::msgs::codec
