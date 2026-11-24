#pragma once

#include "krosc/viz/viz_plugin.hpp"

namespace krosc_ws::viz {

// 프로젝트 전용 Viz 플러그인 예제. 이 파일을 복사해서 자기 패널/툴을 만들면 된다.
class ProjectPanelPlugin final : public krosc::viz::IVizPlugin {
 public:
  [[nodiscard]] krosc::viz::VizPluginInfo info() const override {
    return {"project_panel", "Project Panel", "1.0", {}};
  }
  void initialize(krosc::viz::VizHost& host) override;
};

}  // namespace krosc_ws::viz
