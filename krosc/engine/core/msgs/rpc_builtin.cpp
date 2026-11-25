#include "krosc/net/rpc.hpp"

namespace krosc::msgs::codec {
void register_basic_rpc_codecs();
void register_nav2_rpc_codecs();
void register_parameter_rpc_codecs();
void register_controller_rpc_codecs();
void register_action_rpc_codecs();
void register_arm_rpc_codecs();
}  // namespace krosc::msgs::codec

namespace krosc::net {

void register_builtin_rpc_codecs() {
  msgs::codec::register_basic_rpc_codecs();
  msgs::codec::register_nav2_rpc_codecs();
  msgs::codec::register_parameter_rpc_codecs();
  msgs::codec::register_controller_rpc_codecs();
  msgs::codec::register_action_rpc_codecs();
  msgs::codec::register_arm_rpc_codecs();
}

void register_builtin_action_codecs() {
  msgs::codec::register_action_rpc_codecs();
}

}  // namespace krosc::net
