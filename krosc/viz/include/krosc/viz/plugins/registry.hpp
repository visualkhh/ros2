#pragma once

#include <string>
#include <vector>

#include "krosc/viz/viz_plugin.hpp"

namespace krosc::viz {

// ids에 해당하는 코어 플러그인만 등록한다. 빈 목록이면 아무것도 등록하지 않는다.
void register_core_plugins(VizPluginRegistry& registry, const std::vector<std::string>& ids);

}  // namespace krosc::viz
