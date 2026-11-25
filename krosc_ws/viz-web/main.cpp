#include <cstdio>
#include <string>
#include <utility>

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

  krosc::viz_web::VizWeb app(std::move(options), std::move(registry));
  if (!app.initialize(error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  return app.run();
}
