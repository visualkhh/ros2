#pragma once

#include "srf/viz/viz_plugin.hpp"

namespace srf_default_ws::viz {

// 프로젝트 전용 Viz 플러그인 예제. 이 파일을 복사해서 자기 패널/툴을 만들면 된다.
class ProjectPanelPlugin final : public srf::viz::VizPlugin {
 public:
  ProjectPanelPlugin() : srf::viz::VizPlugin({"project_panel", "Project Panel", "1.0", {}}) {}
  void initialize(srf::viz::VizHost& host) override;
};

}  // namespace srf_default_ws::viz
