#include <QApplication>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "krosc/krosc.hpp"
#include "krosc/viz/plugins/registry.hpp"
#include "krosc/viz/viz.hpp"
#include "krosc/viz/viz_cli.hpp"

#include "plugins/moveit_plugin.hpp"
#include "plugins/ai_plugin.hpp"
#include "plugins/camera_plugin.hpp"
#include "plugins/opencr_plugin.hpp"
#include "plugins/project_panel.hpp"
#include "plugins/rl_nav_plugin.hpp"

int main(int argc, char** argv) {
  QApplication application(argc, argv);

  // 프로젝트 전용 플래그. --no-moveit이 --moveit보다 우선한다.
  bool want_moveit = false;
  bool no_moveit = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--moveit") {
      want_moveit = true;
    } else if (argument == "--no-moveit") {
      no_moveit = true;
    }
  }

  krosc::viz::VizCliDefaults defaults;
  defaults.fixed_frame = "map";
  defaults.tab = "3d";
  defaults.plugins = "project_panel,slam,navigation,moveit,rl_nav,opencr,ai";
  defaults.usage_extra = "[--moveit|--no-moveit]";

  krosc::viz::VizOptions options;
  bool help = false;
  std::string error;
  if (!krosc::viz::parse_viz_options(argc, argv, defaults, options, help, error)) {
    std::fprintf(stderr, "[viz] %s\n", error.c_str());
    return 1;
  }
  if (help) {
    return 0;
  }

  if (no_moveit) {
    options.plugins.erase(
        std::remove(options.plugins.begin(), options.plugins.end(), "moveit"),
        options.plugins.end());
  } else if (want_moveit &&
             std::find(options.plugins.begin(), options.plugins.end(), "moveit") ==
                 options.plugins.end()) {
    options.plugins.push_back("moveit");
  }

  krosc::viz::VizPluginRegistry registry;
  krosc::viz::register_core_plugins(registry, options.plugins);
  const auto selected = [&options](const std::string& id) {
    return std::find(options.plugins.begin(), options.plugins.end(), id) != options.plugins.end();
  };
  if (selected("project_panel")) {
    registry.add("project_panel",
                 [] { return std::make_unique<krosc_ws::viz::ProjectPanelPlugin>(); });
  }
  if (selected("moveit")) {
    registry.add("moveit", [] { return std::make_unique<krosc_ws::viz::MoveItPlugin>(); });
  }
  if (selected("rl_nav")) {
    registry.add("rl_nav", [] { return std::make_unique<krosc_ws::viz::RlNavPlugin>(); });
    registry.add("opencr", [] { return std::make_unique<krosc_ws::viz::OpencrPlugin>(); });
  }
  if (selected("ai")) {
    registry.add("ai", [] { return std::make_unique<krosc_ws::viz::AiPlugin>(); });
  }
  if (selected("camera")) {
    registry.add("camera", [] { return std::make_unique<krosc_ws::viz::CameraPlugin>(); });
  }

  // 원격 Viz도 별도 net::Client 대신 Krosc+PeerNetwork를 쓴다.
  krosc::Krosc app(nullptr, krosc::viz::peer_network_options(options));
  krosc::viz::Viz viz(app.context(), std::move(options), std::move(registry));
  if (!viz.initialize(error)) {
    std::fprintf(stderr, "[viz] %s\n", error.c_str());
    return 1;
  }
  return viz.run();
}
