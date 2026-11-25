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

  // 코어 웹 Viz 플러그인. 프로젝트 전용 플러그인은 각 프로젝트 viz-web main이 추가한다.
  krosc::viz_web::VizWebPluginRegistry registry;
  krosc::viz_web::register_core_web_plugins(registry,
                                            {"slam", "navigation", "teleop", "diagnostics"});

  krosc::viz_web::VizWeb app(std::move(options), std::move(registry));
  if (!app.initialize(error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  return app.run();
}
