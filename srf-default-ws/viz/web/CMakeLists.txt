# srf_default_ws 전용 웹 Viz 앱/플러그인. srf/build에서
#   -DSRF_VIZ_WEB_PROJECT_DIR=<this dir>
# 로 추가된다. HTTP 게이트웨이는 srf::viz_web 코어를 그대로 쓴다.

add_executable(srf_default_ws_viz_web main.cpp)
target_link_libraries(srf_default_ws_viz_web PRIVATE srf::viz_web)
target_compile_definitions(srf_default_ws_viz_web PRIVATE
  SRF_DEFAULT_WS_VIZ_WEB_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/www")
target_compile_options(srf_default_ws_viz_web PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(srf_default_ws_viz_web PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/viz/web"
)
