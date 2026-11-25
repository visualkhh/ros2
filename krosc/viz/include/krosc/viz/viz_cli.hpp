#pragma once

#include <string>

#include "krosc/viz/viz.hpp"

namespace krosc::viz {

// 앱별 기본값. 인자 파싱은 공용으로 하고, 앱은 자기 기본값만 넘긴다.
struct VizCliDefaults {
  std::string fixed_frame = "odom";
  std::string tab = "3d";
  std::string plugins = "slam,navigation";
  std::string usage_extra;  // 앱 전용 옵션 설명 (예: "[--moveit|--no-moveit]")
};

// argc/argv에서 Viz 실행 옵션만 파싱한다.
// --help면 help=true로 두고 true를 반환한다. 잘못된 값이면 false + error.
bool parse_viz_options(int argc, char** argv, const VizCliDefaults& defaults, VizOptions& options,
                       bool& help, std::string& error);

}  // namespace krosc::viz
