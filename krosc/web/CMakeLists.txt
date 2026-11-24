find_package(Threads REQUIRED)

add_executable(krosc_web
  main.cpp
  core/http_server.cpp
  core/robot_link.cpp
  core/websocket.cpp
)

target_include_directories(krosc_web PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(krosc_web PRIVATE Threads::Threads)
target_compile_definitions(krosc_web PRIVATE KROSC_WEB_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/www")
target_compile_options(krosc_web PRIVATE -Wall -Wextra -Wpedantic)
