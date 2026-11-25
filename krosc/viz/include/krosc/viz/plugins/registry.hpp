#pragma once

#include "krosc/viz/viz_plugin.hpp"

namespace krosc::viz {

// 코어 Viz 기본 플러그인(teleop/slam/navigation/manipulator)을 등록한다.
void register_core_plugins(VizPluginRegistry& registry);

}  // namespace krosc::viz
