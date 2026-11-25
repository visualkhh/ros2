find_package(Threads REQUIRED)

add_executable(krosc_viz_web
  main.cpp
  core/viz_web.cpp
  core/viz_web_cli.cpp
  core/viz_web_plugins.cpp
  core/http_server.cpp
  core/robot_link.cpp
  core/websocket.cpp
)

target_include_directories(krosc_viz_web PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(krosc_viz_web PRIVATE Threads::Threads)
target_compile_definitions(krosc_viz_web PRIVATE
  KROSC_VIZ_WEB_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/www")
target_compile_options(krosc_viz_web PRIVATE -Wall -Wextra -Wpedantic)
