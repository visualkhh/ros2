#pragma once

#include <QColor>

#include <algorithm>
#include <cmath>

namespace krosc::viz {

// RViz Map display의 "costmap" color scheme.
//   0       : free (투명 — 아래 지도가 보이게)
//   1..98   : inflation, blue(낮음) -> red(높음)
//   99      : inscribed obstacle (cyan)
//   100     : lethal obstacle (purple)
//   -1      : unknown (blueish greenish gray)
inline QColor costmap_color(int value) {
  if (value < 0) {
    return QColor(90, 120, 120, 80);
  }
  if (value == 0) {
    return QColor(0, 0, 0, 0);
  }
  if (value >= 100) {
    return QColor(255, 0, 255, 225);  // lethal: purple
  }
  if (value == 99) {
    return QColor(0, 255, 255, 220);  // inscribed: cyan
  }
  const double t = static_cast<double>(value) / 99.0;
  const int red = static_cast<int>(std::lround(255.0 * t));
  const int blue = static_cast<int>(std::lround(255.0 * (1.0 - t)));
  return QColor(red, 0, blue, 190);
}

}  // namespace krosc::viz
