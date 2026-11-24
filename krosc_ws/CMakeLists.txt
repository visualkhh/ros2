cmake_minimum_required(VERSION 3.25)
project(krosc_ws VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# krosc core는 프레임워크/노드만 빌드한다 (sim/viz/Qt 없음).
set(KROSC_BUILD_TESTS OFF)
set(KROSC_BUILD_BENCH OFF)
set(KROSC_BUILD_VIZ OFF)
set(KROSC_BUILD_EXAMPLES OFF)
add_subdirectory(../krosc/core core_build)

add_executable(krosc_real
  main.cpp
  robot_bridge.cpp
  robot/real_turtlebot3.cpp
  hardwares/lds03.cpp
  hardwares/opencr_protocol.cpp
  hardwares/opencr_serial.cpp
  hardwares/serial.cpp
)

target_include_directories(krosc_real PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(krosc_real PRIVATE krosc::core)
target_compile_definitions(krosc_real PRIVATE
  KROSC_WS_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/robot/description/urdf/turtlebot3_real_arm.urdf"
  KROSC_WS_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/artifacts/maps"
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(krosc_real PRIVATE -Wno-psabi)
endif()
target_compile_options(krosc_real PRIVATE -Wall -Wextra -Wpedantic)

option(KROSC_WS_BUILD_TESTS "Build krosc_ws tests" ON)
if(KROSC_WS_BUILD_TESTS)
  enable_testing()
  add_executable(krosc_ws_tests
    tests/test_protocol.cpp
    tests/test_serial.cpp
    hardwares/lds03.cpp
    hardwares/opencr_protocol.cpp
    hardwares/opencr_serial.cpp
    hardwares/serial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../krosc/core/tests/test_main.cpp
  )
  target_include_directories(krosc_ws_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../krosc/core/tests
  )
  target_link_libraries(krosc_ws_tests PRIVATE krosc::core)
  if(UNIX AND NOT APPLE)
    target_link_libraries(krosc_ws_tests PRIVATE util)
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(krosc_ws_tests PRIVATE -Wno-psabi)
  endif()
  target_compile_options(krosc_ws_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME krosc_ws_tests COMMAND krosc_ws_tests)
endif()
