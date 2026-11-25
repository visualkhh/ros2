#include "krosc/net/rpc_codec.hpp"

#include "krosc/msgs.hpp"

namespace krosc::msgs::codec {

void register_basic_rpc_codecs() {
  net::register_rpc_codec<msgs::Empty>(
      "std_msgs/Empty", [](net::RpcWriter&, const msgs::Empty&) {},
      [](net::RpcReader&, msgs::Empty&) { return true; });
  net::register_rpc_codec<msgs::String>(
      "std_msgs/String",
      [](net::RpcWriter& w, const msgs::String& value) { w.str(value.data); },
      [](net::RpcReader& r, msgs::String& value) {
        value.data = r.str();
        return true;
      });
  net::register_rpc_codec<msgs::Bool>(
      "std_msgs/Bool",
      [](net::RpcWriter& w, const msgs::Bool& value) { w.u8(value.data ? 1U : 0U); },
      [](net::RpcReader& r, msgs::Bool& value) {
        value.data = r.u8() != 0;
        return true;
      });

  net::register_rpc_codec<msgs::SetBoolRequest>(
      "std_srvs/SetBool_Request",
      [](net::RpcWriter& w, const msgs::SetBoolRequest& value) {
        w.u8(value.data ? 1U : 0U);
      },
      [](net::RpcReader& r, msgs::SetBoolRequest& value) {
        value.data = r.u8() != 0;
        return true;
      });
  net::register_rpc_codec<msgs::SetBoolResponse>(
      "std_srvs/SetBool_Response",
      [](net::RpcWriter& w, const msgs::SetBoolResponse& value) {
        w.u8(value.success ? 1U : 0U);
        w.str(value.message);
      },
      [](net::RpcReader& r, msgs::SetBoolResponse& value) {
        value.success = r.u8() != 0;
        value.message = r.str();
        return true;
      });
  net::register_rpc_codec<msgs::TriggerRequest>(
      "std_srvs/Trigger_Request", [](net::RpcWriter&, const msgs::TriggerRequest&) {},
      [](net::RpcReader&, msgs::TriggerRequest&) { return true; });
  net::register_rpc_codec<msgs::TriggerResponse>(
      "std_srvs/Trigger_Response",
      [](net::RpcWriter& w, const msgs::TriggerResponse& value) {
        w.u8(value.success ? 1U : 0U);
        w.str(value.message);
      },
      [](net::RpcReader& r, msgs::TriggerResponse& value) {
        value.success = r.u8() != 0;
        value.message = r.str();
        return true;
      });
}

}  // namespace krosc::msgs::codec
