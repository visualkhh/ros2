#include <QApplication>

#include <cstdio>
#include <string>
#include <utility>

#include "krosc/krosc.hpp"
#include "krosc/viz/plugins/registry.hpp"
#include "krosc/viz/viz.hpp"
#include "krosc/viz/viz_cli.hpp"

int main(int argc, char** argv) {
  QApplication application(argc, argv);

  krosc::viz::VizCliDefaults defaults;
  defaults.fixed_frame = "odom";
  defaults.tab = "3d";
  defaults.plugins = "slam,navigation";

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

  krosc::viz::VizPluginRegistry registry;
  krosc::viz::register_core_plugins(registry, options.plugins);

  // 원격 Viz도 별도 net::Client 대신 Krosc+PeerNetwork를 쓴다.
  krosc::Krosc app(nullptr, krosc::viz::peer_network_options(options));
  krosc::viz::Viz viz(app.context(), std::move(options), std::move(registry));
  if (!viz.initialize(error)) {
    std::fprintf(stderr, "[viz] %s\n", error.c_str());
    return 1;
  }
  return viz.run();
}
