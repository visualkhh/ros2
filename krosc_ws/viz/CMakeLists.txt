# krosc_ws 전용 Viz 플러그인/앱. krosc/build에서
#   -DKROSC_VIZ_PROJECT_DIR=<this dir>
# 로 추가된다. Pi의 krosc_ws 빌드에는 Qt가 필요 없다.

add_library(krosc_ws_viz_plugins STATIC
  plugins/project_panel.cpp
  plugins/moveit_plugin.cpp
  plugins/rl_nav_plugin.cpp
  plugins/opencr_plugin.cpp
  plugins/ai_plugin.cpp
  plugins/camera_plugin.cpp
)
target_include_directories(krosc_ws_viz_plugins PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(krosc_ws_viz_plugins PUBLIC krosc::viz krosc::viz_plugins)
target_compile_options(krosc_ws_viz_plugins PRIVATE -Wall -Wextra -Wpedantic)

add_executable(krosc_ws_viz_app main.cpp)
target_link_libraries(krosc_ws_viz_app PRIVATE krosc::viz krosc_ws_viz_plugins)
target_compile_options(krosc_ws_viz_app PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(krosc_ws_viz_app PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/viz"
)
