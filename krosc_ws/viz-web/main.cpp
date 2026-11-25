#include <cstdio>
#include <string>
#include <utility>

#include "krosc/krosc.hpp"
#include "krosc/viz_web/viz_web.hpp"
#include "krosc/viz_web/viz_web_cli.hpp"
#include "krosc/viz_web/viz_web_plugins.hpp"

#ifndef KROSC_WS_VIZ_WEB_ROOT
#define KROSC_WS_VIZ_WEB_ROOT "viz-web/www"
#endif

int main(int argc, char** argv) {
  krosc::viz_web::VizWebOptions options;
  options.overlay_root = KROSC_WS_VIZ_WEB_ROOT;

  bool help = false;
  std::string error;
  if (!krosc::viz_web::parse_viz_web_options(argc, argv, options, help, error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  if (help) {
    return 0;
  }

  // 코어 플러그인 + krosc_ws 프로젝트 플러그인.
  krosc::viz_web::VizWebPluginRegistry registry;
  krosc::viz_web::register_core_web_plugins(registry,
                                            {"slam", "navigation", "teleop", "diagnostics"});
  registry.add("moveit", "MoveIt", "/plugins/moveit.js");
  registry.add("rl_nav", "RL Nav", "/plugins/rl_nav.js");

  // 프로젝트 플러그인이 쓰는 토픽. 코어 웹 게이트웨이는 기본 토픽만 안다.
  options.forward_topics = {
      {"arm_status", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"moveit_status", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"moveit_state", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"moveit_target_pose", krosc::net::Kind::Pose, krosc::QOS_DEFAULT},
      {"moveit_arm", krosc::net::Kind::Path, krosc::QOS_DEFAULT},
      {"moveit_path", krosc::net::Kind::Path, krosc::QOS_DEFAULT},
      {"rl_nav_status", krosc::net::Kind::String, krosc::QOS_LATCHED},
      {"rl_nav_model_list", krosc::net::Kind::String, krosc::QOS_LATCHED},
  };

  krosc::net::PeerNetworkOptions network;
  network.name = "krosc_ws_viz_web";
  if (!options.robot_host.empty()) {
    network.add_server(options.robot_host, options.robot_port);
  }
  krosc::Krosc app(nullptr, network);
  krosc::viz_web::VizWeb web(std::move(options), std::move(registry), app.context());
  if (!web.initialize(error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  return web.run();
}
