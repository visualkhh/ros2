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
add_subdirectory(../krosc/engine engine_build)

set(KROSC_REAL_SOURCES
  robot/main.cpp
  robot/turtlebot3_driver.cpp
  robot/real_turtlebot3.cpp
  robot/hardware/opencr_system.cpp
  robot/hardwares/lds03.cpp
  robot/hardwares/opencr_protocol.cpp
  robot/hardwares/opencr_serial.cpp
  robot/hardwares/serial.cpp
)

# RL nav: ONNX Runtime은 ROS 2 lyrical vendor를 우선 찾고, 없으면 RL만 제외한다.
option(KROSC_WS_BUILD_RL_NAV "Build ONNX RL navigation controller" ON)
set(KROSC_WS_ONNXRUNTIME_ROOT "/opt/ros/lyrical/opt/onnxruntime_vendor" CACHE PATH
    "ONNX Runtime vendor root (include/, lib/)")
set(KROSC_WS_RL_NAV_FOUND OFF)
if(KROSC_WS_BUILD_RL_NAV)
  find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS "${KROSC_WS_ONNXRUNTIME_ROOT}" ENV ONNXRUNTIME_ROOT
    PATH_SUFFIXES include)
  find_library(ONNXRUNTIME_LIBRARY onnxruntime
    HINTS "${KROSC_WS_ONNXRUNTIME_ROOT}" ENV ONNXRUNTIME_ROOT
    PATH_SUFFIXES lib)
  if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
    list(APPEND KROSC_REAL_SOURCES
      robot/ml/onnx_policy.cpp
      robot/nodes/rl_nav_controller.cpp)
    set(KROSC_WS_RL_NAV_FOUND ON)
  else()
    message(STATUS "krosc_ws: ONNX Runtime을 찾지 못해 RL nav를 제외한다")
  endif()
endif()

add_executable(krosc_real ${KROSC_REAL_SOURCES})

target_include_directories(krosc_real PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/robot
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(krosc_real PRIVATE krosc::core)
target_compile_definitions(krosc_real PRIVATE
  KROSC_WS_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/robot/description/urdf/turtlebot3_real.urdf"
  KROSC_WS_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/artifacts/maps"
  KROSC_WS_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/artifacts/models"
)
if(KROSC_WS_RL_NAV_FOUND)
  target_include_directories(krosc_real PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(krosc_real PRIVATE ${ONNXRUNTIME_LIBRARY})
  target_compile_definitions(krosc_real PRIVATE KROSC_WS_ENABLE_RL_NAV=1)
  get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
  set_target_properties(krosc_real PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
    INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")
endif()
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
    tests/test_moveit_stack.cpp
    tests/test_models.cpp
    robot/hardwares/lds03.cpp
    robot/hardwares/opencr_protocol.cpp
    robot/hardwares/opencr_serial.cpp
    robot/hardwares/serial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../krosc/engine/tests/test_main.cpp
  )
  target_include_directories(krosc_ws_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/robot
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../krosc/engine/tests
  )
  target_link_libraries(krosc_ws_tests PRIVATE krosc::core)
  target_compile_definitions(krosc_ws_tests PRIVATE
    KROSC_WS_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/robot/description/urdf/turtlebot3_real.urdf")
  if(UNIX AND NOT APPLE)
    target_link_libraries(krosc_ws_tests PRIVATE util)
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(krosc_ws_tests PRIVATE -Wno-psabi)
  endif()
  target_compile_options(krosc_ws_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME krosc_ws_tests COMMAND krosc_ws_tests)
endif()
