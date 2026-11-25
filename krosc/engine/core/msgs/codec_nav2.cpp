#include "krosc/net/rpc_codec.hpp"

#include "krosc/msgs/nav2_srv.hpp"

namespace krosc::msgs::codec {

void register_nav2_rpc_codecs() {
  net::register_rpc_codec<nav2::LoadMapRequest>(
      "nav2_msgs/srv/LoadMap_Request",
      [](net::RpcWriter& w, const nav2::LoadMapRequest& value) { w.str(value.map_url); },
      [](net::RpcReader& r, nav2::LoadMapRequest& value) {
        value.map_url = r.str();
        return true;
      });
  net::register_rpc_codec<nav2::LoadMapResponse>(
      "nav2_msgs/srv/LoadMap_Response",
      [](net::RpcWriter& w, const nav2::LoadMapResponse& value) {
        w.u8(static_cast<std::uint8_t>(value.result));
      },
      [](net::RpcReader& r, nav2::LoadMapResponse& value) {
        value.result = r.u8();
        return true;
      });
  net::register_rpc_codec<nav2::SaveMapRequest>(
      "nav2_msgs/srv/SaveMap_Request",
      [](net::RpcWriter& w, const nav2::SaveMapRequest& value) {
        w.str(value.map_topic);
        w.str(value.map_url);
        w.str(value.image_format);
        w.str(value.map_mode);
        w.f64(value.free_thresh);
        w.f64(value.occupied_thresh);
      },
      [](net::RpcReader& r, nav2::SaveMapRequest& value) {
        value.map_topic = r.str();
        value.map_url = r.str();
        value.image_format = r.str();
        value.map_mode = r.str();
        value.free_thresh = r.f64();
        value.occupied_thresh = r.f64();
        return true;
      });
  net::register_rpc_codec<nav2::SaveMapResponse>(
      "nav2_msgs/srv/SaveMap_Response",
      [](net::RpcWriter& w, const nav2::SaveMapResponse& value) {
        w.u8(value.result ? 1U : 0U);
      },
      [](net::RpcReader& r, nav2::SaveMapResponse& value) {
        value.result = r.u8() != 0;
        return true;
      });
  net::register_rpc_codec<nav2::DeleteMapRequest>(
      "nav2_msgs/srv/DeleteMap_Request",
      [](net::RpcWriter& w, const nav2::DeleteMapRequest& value) { w.str(value.map_url); },
      [](net::RpcReader& r, nav2::DeleteMapRequest& value) {
        value.map_url = r.str();
        return true;
      });
  net::register_rpc_codec<nav2::DeleteMapResponse>(
      "nav2_msgs/srv/DeleteMap_Response",
      [](net::RpcWriter& w, const nav2::DeleteMapResponse& value) {
        w.u8(value.result ? 1U : 0U);
        w.str(value.message);
      },
      [](net::RpcReader& r, nav2::DeleteMapResponse& value) {
        value.result = r.u8() != 0;
        value.message = r.str();
        return true;
      });
}

}  // namespace krosc::msgs::codec
