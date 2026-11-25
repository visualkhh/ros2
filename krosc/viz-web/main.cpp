#include <cstdio>
#include <string>
#include <utility>

#include "krosc/viz_web/viz_web.hpp"
#include "krosc/viz_web/viz_web_cli.hpp"
#include "krosc/viz_web/viz_web_plugins.hpp"

int main(int argc, char** argv) {
  krosc::viz_web::VizWebOptions options;
  bool help = false;
  std::string error;
  if (!krosc::viz_web::parse_viz_web_options(argc, argv, options, help, error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  if (help) {
    return 0;
  }

  // 기본 웹 Viz 플러그인. 필요한 것만 추가/제거하면 /plugins.json에 바로 반영된다.
  krosc::viz_web::VizWebPluginRegistry registry;
  registry.add("slam", "SLAM / Map", "/plugins/slam.js");
  registry.add("navigation", "Navigation", "/plugins/navigation.js");
  registry.add("teleop", "Teleop", "/plugins/teleop.js");
  registry.add("rl_nav", "RL Nav", "/plugins/rl_nav.js");
  registry.add("moveit", "MoveIt", "/plugins/moveit.js");
  registry.add("diagnostics", "Diagnostics", "/plugins/diagnostics.js");

  krosc::viz_web::VizWeb app(std::move(options), std::move(registry));
  if (!app.initialize(error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  return app.run();
}
