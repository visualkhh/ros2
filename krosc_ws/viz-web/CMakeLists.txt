# krosc_ws 전용 웹 Viz 앱/플러그인. krosc/build에서
#   -DKROSC_VIZ_WEB_PROJECT_DIR=<this dir>
# 로 추가된다. HTTP 게이트웨이는 krosc::viz_web 코어를 그대로 쓴다.

add_executable(krosc_ws_viz_web main.cpp)
target_link_libraries(krosc_ws_viz_web PRIVATE krosc::viz_web)
target_compile_definitions(krosc_ws_viz_web PRIVATE
  KROSC_WS_VIZ_WEB_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/www")
target_compile_options(krosc_ws_viz_web PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(krosc_ws_viz_web PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/viz-web"
)
