#include "krosc/net/rpc_codec.hpp"

#include "krosc/msgs.hpp"

namespace krosc::msgs::codec {

void register_arm_rpc_codecs() {
  net::register_rpc_codec<msgs::ArmResetRequest>(
      "krosc_ws/srv/ArmReset_Request",
      [](net::RpcWriter& w, const msgs::ArmResetRequest& value) { w.str(value.target); },
      [](net::RpcReader& r, msgs::ArmResetRequest& value) {
        value.target = r.str();
        return true;
      });
  net::register_rpc_codec<msgs::ArmResetResponse>(
      "krosc_ws/srv/ArmReset_Response",
      [](net::RpcWriter& w, const msgs::ArmResetResponse& value) {
        w.u8(value.success ? 1U : 0U);
        w.str(value.message);
      },
      [](net::RpcReader& r, msgs::ArmResetResponse& value) {
        value.success = r.u8() != 0;
        value.message = r.str();
        return true;
      });
  net::register_rpc_codec<msgs::BuzzerRequest>(
      "krosc_ws/srv/Buzzer_Request",
      [](net::RpcWriter& w, const msgs::BuzzerRequest& value) {
        w.u32(static_cast<std::uint32_t>(value.frequency));
        w.u32(static_cast<std::uint32_t>(value.duration));
      },
      [](net::RpcReader& r, msgs::BuzzerRequest& value) {
        value.frequency = static_cast<std::int32_t>(r.u32());
        value.duration = static_cast<std::int32_t>(r.u32());
        return true;
      });
  net::register_rpc_codec<msgs::BuzzerResponse>(
      "krosc_ws/srv/Buzzer_Response",
      [](net::RpcWriter& w, const msgs::BuzzerResponse& value) {
        w.u8(value.success ? 1U : 0U);
        w.str(value.message);
      },
      [](net::RpcReader& r, msgs::BuzzerResponse& value) {
        value.success = r.u8() != 0;
        value.message = r.str();
        return true;
      });
}

}  // namespace krosc::msgs::codec
