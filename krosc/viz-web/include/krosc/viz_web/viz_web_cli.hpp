#pragma once

#include <string>

#include "krosc/viz_web/viz_web.hpp"

namespace krosc::viz_web {

// argc/argv에서 웹 Viz 실행 옵션만 파싱한다.
// --help면 help=true로 두고 true를 반환한다. 잘못된 값이면 false + error.
bool parse_viz_web_options(int argc, char** argv, VizWebOptions& options, bool& help,
                           std::string& error);

}  // namespace krosc::viz_web
