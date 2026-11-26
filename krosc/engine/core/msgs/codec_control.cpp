#include "krosc/net/rpc_codec.hpp"

#include "krosc/control/controller_manager.hpp"

#include "codec_util.hpp"

namespace krosc::msgs::codec {

void register_controller_rpc_codecs() {
  net::register_rpc_codec<krosc::control::ListControllersRequest>(
      "controller_manager_msgs/srv/ListControllers_Request",
      [](net::RpcWriter&, const krosc::control::ListControllersRequest&) {},
      [](net::RpcReader&, krosc::control::ListControllersRequest&) { return true; });
  net::register_rpc_codec<krosc::control::ListControllersResponse>(
      "controller_manager_msgs/srv/ListControllers_Response",
      [](net::RpcWriter& w, const krosc::control::ListControllersResponse& value) {
        encode_strings(w, value.name);
        encode_strings(w, value.type);
        encode_strings(w, value.state);
      },
      [](net::RpcReader& r, krosc::control::ListControllersResponse& value) {
        value.name = decode_strings(r);
        value.type = decode_strings(r);
        value.state = decode_strings(r);
        return true;
      });
  net::register_rpc_codec<krosc::control::LoadControllerRequest>(
      "controller_manager_msgs/srv/LoadController_Request",
      [](net::RpcWriter& w, const krosc::control::LoadControllerRequest& value) {
        w.str(value.name);
      },
      [](net::RpcReader& r, krosc::control::LoadControllerRequest& value) {
        value.name = r.str();
        return true;
      });
  net::register_rpc_codec<krosc::control::LoadControllerResponse>(
      "controller_manager_msgs/srv/LoadController_Response",
      [](net::RpcWriter& w, const krosc::control::LoadControllerResponse& value) {
        w.u8(value.ok ? 1U : 0U);
        w.str(value.message);
      },
      [](net::RpcReader& r, krosc::control::LoadControllerResponse& value) {
        value.ok = r.u8() != 0;
        value.message = r.str();
        return true;
      });
  net::register_rpc_codec<krosc::control::ConfigureControllerRequest>(
      "controller_manager_msgs/srv/ConfigureController_Request",
      [](net::RpcWriter& w, const krosc::control::ConfigureControllerRequest& value) {
        w.str(value.name);
      },
      [](net::RpcReader& r, krosc::control::ConfigureControllerRequest& value) {
        value.name = r.str();
        return true;
      });
  net::register_rpc_codec<krosc::control::ConfigureControllerResponse>(
      "controller_manager_msgs/srv/ConfigureController_Response",
      [](net::RpcWriter& w, const krosc::control::ConfigureControllerResponse& value) {
        w.u8(value.ok ? 1U : 0U);
        w.str(value.message);
      },
      [](net::RpcReader& r, krosc::control::ConfigureControllerResponse& value) {
        value.ok = r.u8() != 0;
        value.message = r.str();
        return true;
      });
  net::register_rpc_codec<krosc::control::SwitchControllerRequest>(
      "controller_manager_msgs/srv/SwitchController_Request",
      [](net::RpcWriter& w, const krosc::control::SwitchControllerRequest& value) {
        encode_strings(w, value.start_controllers);
        encode_strings(w, value.stop_controllers);
        w.i64(value.strictness);
        w.u8(value.start_asap ? 1U : 0U);
        w.f64(value.timeout);
      },
      [](net::RpcReader& r, krosc::control::SwitchControllerRequest& value) {
        value.start_controllers = decode_strings(r);
        value.stop_controllers = decode_strings(r);
        value.strictness = r.i64();
        value.start_asap = r.u8() != 0;
        value.timeout = r.f64();
        return true;
      });
  net::register_rpc_codec<krosc::control::SwitchControllerResponse>(
      "controller_manager_msgs/srv/SwitchController_Response",
      [](net::RpcWriter& w, const krosc::control::SwitchControllerResponse& value) {
        w.u8(value.ok ? 1U : 0U);
        w.str(value.message);
      },
      [](net::RpcReader& r, krosc::control::SwitchControllerResponse& value) {
        value.ok = r.u8() != 0;
        value.message = r.str();
        return true;
      });
}

}  // namespace krosc::msgs::codec
