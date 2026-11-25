find_package(Threads REQUIRED)

# 웹 Viz 코어: 게이트웨이/레지스트리/CLI. 프로젝트 앱이 링크해서 쓴다.
add_library(krosc_viz_web_core STATIC
  core/viz_web.cpp
  core/viz_web_cli.cpp
  core/viz_web_plugins.cpp
  core/context_link.cpp
  core/http_server.cpp
  core/json.cpp
  core/web_rpc.cpp
  core/websocket.cpp
)
add_library(krosc::viz_web ALIAS krosc_viz_web_core)

target_include_directories(krosc_viz_web_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(krosc_viz_web_core PUBLIC Threads::Threads krosc::core)
target_compile_definitions(krosc_viz_web_core PUBLIC
  KROSC_VIZ_WEB_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/www")
target_compile_options(krosc_viz_web_core PRIVATE -Wall -Wextra -Wpedantic)

# 코어 웹 Viz 실행기: 프로젝트 플러그인 없이 기본 게이트웨이만 띄운다.
add_executable(krosc_viz_web main.cpp)
target_link_libraries(krosc_viz_web PRIVATE krosc::viz_web)
target_compile_options(krosc_viz_web PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(krosc_viz_web PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/viz-web"
)

# 프로젝트별 웹 Viz 앱 (예: krosc_ws/viz-web). 프로젝트가 자기 main/JS를 가진다.
set(KROSC_VIZ_WEB_PROJECT_DIR "" CACHE PATH "Optional project-specific viz-web plugin/app directory")
if(KROSC_VIZ_WEB_PROJECT_DIR)
  add_subdirectory(${KROSC_VIZ_WEB_PROJECT_DIR} ${CMAKE_CURRENT_BINARY_DIR}/project)
endif()

install(TARGETS krosc_viz_web_core ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY www/ DESTINATION share/krosc/viz-web)
