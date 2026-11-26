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
  engine/main.cpp
  engine/engine.cpp
  engine/nodes/robot_status.cpp
  engine/nodes/llm_input.cpp
  engine/nodes/llm_client.cpp
  engine/nodes/llm_json.cpp
  engine/nodes/command_rules.cpp
  engine/nodes/http_client.cpp
  engine/nodes/vla_node.cpp
  engine/nodes/vlm_node.cpp
  engine/nodes/pick_place_node.cpp
  engine/nodes/nav_behavior_node.cpp
  engine/controls/opencr/opencr_control.cpp
  engine/hardwares/lds03/lds03_device.cpp
  engine/sensors/lds03/lds03_sensor.cpp
  engine/sensors/opencr/opencr_imu.cpp
  engine/sensors/realsense/realsense_sensor.cpp
  engine/hardwares/opencr/opencr_protocol.cpp
  engine/hardwares/opencr/opencr_serial.cpp
  engine/hardwares/serial.cpp
)

# RealSense D435: librealsense2 + libjpeg가 있을 때만 Hardware/Sensor를 포함한다.
# 없으면 krosc_real은 카메라 없이 빌드되고 --no-camera와 같은 상태가 된다.
# ROS 경로를 하드코딩하지 않는다. standalone prefix는 KROSC_WS_REALSENSE_ROOT,
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH 중 하나로 지정한다.
option(KROSC_WS_BUILD_REALSENSE "Build RealSense D435 hardware/sensor" ON)
set(KROSC_WS_REALSENSE_ROOT "" CACHE PATH
    "librealsense2 install prefix (optional; CMake/pkg-config가 찾지 못할 때)")
set(KROSC_WS_REALSENSE_FOUND OFF)
if(KROSC_WS_BUILD_REALSENSE)
  if(KROSC_WS_REALSENSE_ROOT)
    list(PREPEND CMAKE_PREFIX_PATH "${KROSC_WS_REALSENSE_ROOT}")
    set(ENV{PKG_CONFIG_PATH}
        "${KROSC_WS_REALSENSE_ROOT}/lib/pkgconfig:${KROSC_WS_REALSENSE_ROOT}/lib/aarch64-linux-gnu/pkgconfig:$ENV{PKG_CONFIG_PATH}")
  endif()
  find_package(realsense2 QUIET)
  if(NOT realsense2_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(KROSC_RS2 QUIET IMPORTED_TARGET realsense2)
    endif()
  endif()
  if(realsense2_FOUND OR KROSC_RS2_FOUND)
    find_package(JPEG QUIET)
    if(JPEG_FOUND)
      list(APPEND KROSC_REAL_SOURCES
        engine/hardwares/realsense/realsense_device.cpp)
      set(KROSC_WS_REALSENSE_FOUND ON)
      message(STATUS "krosc_ws: RealSense D435 활성 (librealsense2 + libjpeg)")
    else()
      message(STATUS "krosc_ws: libjpeg 없음 — RealSense 비활성")
    endif()
  else()
    message(STATUS "krosc_ws: librealsense2 없음 — RealSense 비활성")
  endif()
endif()

# RL nav: ONNX Runtime을 찾으면 GPU(CUDA/CoreML) → CPU 순으로 쓴다.
# 못 찾아도 RL 컨트롤러 플러그인은 빌드한다. 정책 없이 fallback 컨트롤러로 주행하고
# diagnostics에 onnxruntime=false를 남긴다.
option(KROSC_WS_BUILD_RL_NAV "Build ONNX RL navigation controller" ON)
if(DEFINED ENV{ONNXRUNTIME_ROOT} AND EXISTS "$ENV{ONNXRUNTIME_ROOT}")
  set(_krosc_ws_ort_default "$ENV{ONNXRUNTIME_ROOT}")
elseif(EXISTS "$ENV{HOME}/opt/onnxruntime-gpu/1.26.0")
  set(_krosc_ws_ort_default "$ENV{HOME}/opt/onnxruntime-gpu/1.26.0")
else()
  set(_krosc_ws_ort_default "")
endif()
set(KROSC_WS_ONNXRUNTIME_ROOT "${_krosc_ws_ort_default}" CACHE PATH
    "ONNX Runtime vendor root (include/, lib/)")
set(KROSC_WS_RL_NAV_FOUND OFF)
if(KROSC_WS_BUILD_RL_NAV)
  list(APPEND KROSC_REAL_SOURCES
    engine/ml/onnx_policy.cpp
    engine/nodes/rl_nav_controller.cpp)
  find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS "${KROSC_WS_ONNXRUNTIME_ROOT}" ENV ONNXRUNTIME_ROOT ENV HOMEBREW_PREFIX
    PATH_SUFFIXES include include/onnxruntime)
  find_library(ONNXRUNTIME_LIBRARY onnxruntime
    HINTS "${KROSC_WS_ONNXRUNTIME_ROOT}" ENV ONNXRUNTIME_ROOT ENV HOMEBREW_PREFIX
    PATH_SUFFIXES lib)
  if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
    set(KROSC_WS_RL_NAV_FOUND ON)
  else()
    message(STATUS "krosc_ws: ONNX Runtime 없음 — RL은 CPU fallback 컨트롤러로만 동작한다")
  endif()
endif()

add_executable(krosc_real ${KROSC_REAL_SOURCES})

target_include_directories(krosc_real PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/engine
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(krosc_real PRIVATE krosc::core)
if(KROSC_WS_REALSENSE_FOUND)
  set(_krosc_rs2_target "")
  if(TARGET realsense2::realsense2)
    set(_krosc_rs2_target realsense2::realsense2)
  elseif(TARGET PkgConfig::KROSC_RS2)
    set(_krosc_rs2_target PkgConfig::KROSC_RS2)
  endif()
  target_link_libraries(krosc_real PRIVATE ${_krosc_rs2_target} JPEG::JPEG)
  target_compile_definitions(krosc_real PRIVATE KROSC_WS_HAS_REALSENSE=1)

  set(_krosc_rs2_rpath_dirs "")
  if(TARGET realsense2::realsense2)
    get_target_property(_krosc_rs2_library realsense2::realsense2 IMPORTED_LOCATION)
    if(NOT _krosc_rs2_library)
      get_target_property(_krosc_rs2_library realsense2::realsense2 IMPORTED_LOCATION_NONE)
    endif()
    if(_krosc_rs2_library)
      get_filename_component(_krosc_rs2_library_dir "${_krosc_rs2_library}" DIRECTORY)
      list(APPEND _krosc_rs2_rpath_dirs "${_krosc_rs2_library_dir}")
    endif()
  elseif(KROSC_RS2_LIBRARY_DIRS)
    list(APPEND _krosc_rs2_rpath_dirs ${KROSC_RS2_LIBRARY_DIRS})
  endif()
  foreach(_krosc_rs2_dir IN LISTS _krosc_rs2_rpath_dirs)
    set_property(TARGET krosc_real APPEND PROPERTY BUILD_RPATH "${_krosc_rs2_dir}")
    set_property(TARGET krosc_real APPEND PROPERTY INSTALL_RPATH "${_krosc_rs2_dir}")
  endforeach()
endif()
target_compile_definitions(krosc_real PRIVATE
  KROSC_WS_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/engine/description/urdf/turtlebot3_real.urdf"
  KROSC_WS_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/artifacts/maps"
  KROSC_WS_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/artifacts/models"
)
if(KROSC_WS_BUILD_RL_NAV)
  target_compile_definitions(krosc_real PRIVATE KROSC_WS_ENABLE_RL_NAV=1)
  if(KROSC_WS_RL_NAV_FOUND)
    target_include_directories(krosc_real PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
    target_link_libraries(krosc_real PRIVATE ${ONNXRUNTIME_LIBRARY})
    if(UNIX AND NOT APPLE)
      target_link_options(krosc_real PRIVATE -Wl,--disable-new-dtags)
    endif()
    target_compile_definitions(krosc_real PRIVATE KROSC_WS_HAS_ONNXRUNTIME=1)
    get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
    set_target_properties(krosc_real PROPERTIES
      BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
      INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")
  endif()
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(krosc_real PRIVATE -Wno-psabi)
endif()
target_compile_options(krosc_real PRIVATE -Wall -Wextra -Wpedantic)

# PC 전용 시뮬레이터. Pi(ARM)에서는 타깃을 만들지 않는다.
if(NOT (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|armv.*)$"))
  add_subdirectory(../krosc/simulator simulator_build)
  # 시뮬레이터 버저 소리: Qt Multimedia가 있으면 실제 스피커로 낸다.
  find_package(Qt6 QUIET COMPONENTS Multimedia)
  # macOS SDK에는 AGL.framework가 없다. Qt가 WrapOpenGL에 다시 넣는 경우가 있어
  # Multimedia find 이후 한 번 더 걷어낸다 (krosc/simulator 쪽 필터와 같은 처리).
  if(APPLE AND TARGET WrapOpenGL::WrapOpenGL)
    get_target_property(_krosc_ws_wrap_libs WrapOpenGL::WrapOpenGL INTERFACE_LINK_LIBRARIES)
    if(_krosc_ws_wrap_libs)
      list(FILTER _krosc_ws_wrap_libs EXCLUDE REGEX "AGL")
      set_target_properties(WrapOpenGL::WrapOpenGL PROPERTIES
        INTERFACE_LINK_LIBRARIES "${_krosc_ws_wrap_libs}")
    endif()
  endif()

  add_executable(krosc_ws_simulator
    simulator/main.cpp
    simulator/hardwares/opencr/simulator_opencr.cpp
    simulator/hardwares/opencr/simulator_buzzer.cpp
    simulator/hardwares/lds03/simulator_lds.cpp
    simulator/robot/waffle_pi_arm.cpp
    simulator/robot/urdf_robot_visualizer.cpp
    simulator/panels/arm_panel.cpp
    simulator/panels/camera_panel.cpp
    engine/engine.cpp
    engine/nodes/robot_status.cpp
    engine/nodes/llm_input.cpp
    engine/nodes/llm_client.cpp
    engine/nodes/llm_json.cpp
    engine/nodes/command_rules.cpp
    engine/nodes/http_client.cpp
    engine/nodes/vla_node.cpp
    engine/nodes/vlm_node.cpp
    engine/nodes/pick_place_node.cpp
    engine/nodes/nav_behavior_node.cpp
    engine/controls/opencr/opencr_control.cpp
    engine/sensors/lds03/lds03_sensor.cpp
    engine/sensors/opencr/opencr_imu.cpp
    engine/sensors/realsense/realsense_sensor.cpp
    simulator/hardwares/realsense/simulator_realsense.cpp
  )
  target_include_directories(krosc_ws_simulator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/engine
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/simulator)
  if(NOT TARGET krosc::simulator_qt)
    message(FATAL_ERROR "krosc_ws_simulator: Qt6 simulator window가 필요하다")
  endif()
  target_link_libraries(krosc_ws_simulator PRIVATE
    krosc::core krosc::simulator_assets krosc::simulator krosc::simulator_qt)
  if(TARGET Qt6::Multimedia)
    target_link_libraries(krosc_ws_simulator PRIVATE Qt6::Multimedia)
    target_compile_definitions(krosc_ws_simulator PRIVATE KROSC_WS_SIM_AUDIO=1)
    message(STATUS "krosc_ws: 시뮬레이터 버저 소리 활성 (Qt6Multimedia)")
  else()
    message(STATUS "krosc_ws: Qt6Multimedia 없음 — 시뮬레이터 버저 소리 비활성")
  endif()
  target_compile_definitions(krosc_ws_simulator PRIVATE
    KROSC_WS_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/simulator/robot/urdf/turtlebot3_real.urdf"
    KROSC_WS_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/artifacts/maps"
    KROSC_WS_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/artifacts/models")
  target_compile_options(krosc_ws_simulator PRIVATE -Wall -Wextra -Wpedantic)
  if(KROSC_WS_BUILD_RL_NAV)
    target_sources(krosc_ws_simulator PRIVATE
      engine/ml/onnx_policy.cpp
      engine/nodes/rl_nav_controller.cpp)
    target_compile_definitions(krosc_ws_simulator PRIVATE KROSC_WS_ENABLE_RL_NAV=1)
    if(KROSC_WS_RL_NAV_FOUND)
      target_include_directories(krosc_ws_simulator PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
      target_link_libraries(krosc_ws_simulator PRIVATE ${ONNXRUNTIME_LIBRARY})
      if(UNIX AND NOT APPLE)
        target_link_options(krosc_ws_simulator PRIVATE -Wl,--disable-new-dtags)
      endif()
      target_compile_definitions(krosc_ws_simulator PRIVATE KROSC_WS_HAS_ONNXRUNTIME=1)
      get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
      set_target_properties(krosc_ws_simulator PROPERTIES
        BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
        INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")
    endif()
  endif()
endif()

option(KROSC_WS_BUILD_TESTS "Build krosc_ws tests" ON)
if(KROSC_WS_BUILD_TESTS)
  enable_testing()
  add_executable(krosc_ws_tests
    tests/test_protocol.cpp
    tests/test_serial.cpp
    tests/test_moveit_stack.cpp
    tests/test_models.cpp
    tests/test_rl_nav_plugin.cpp
    tests/test_llm.cpp
    tests/test_commands.cpp
    engine/nodes/llm_json.cpp
    engine/nodes/command_rules.cpp
    engine/ml/onnx_policy.cpp
    engine/nodes/rl_nav_controller.cpp
    engine/hardwares/lds03/lds03_device.cpp
    engine/hardwares/opencr/opencr_protocol.cpp
    engine/hardwares/opencr/opencr_serial.cpp
    engine/hardwares/serial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../krosc/engine/tests/test_main.cpp
  )
  target_include_directories(krosc_ws_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/engine
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../krosc/engine/tests
  )
  target_link_libraries(krosc_ws_tests PRIVATE krosc::core)
  if(NOT (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|armv.*)$"))
    target_sources(krosc_ws_tests PRIVATE
      tests/test_sim_robot.cpp
      tests/test_sim_realsense.cpp
      simulator/robot/waffle_pi_arm.cpp
      simulator/robot/urdf_robot_visualizer.cpp
      simulator/hardwares/realsense/simulator_realsense.cpp)
    target_include_directories(krosc_ws_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/simulator)
    target_link_libraries(krosc_ws_tests PRIVATE krosc::simulator_assets)
  endif()
  target_compile_definitions(krosc_ws_tests PRIVATE
    KROSC_WS_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/engine/description/urdf/turtlebot3_real.urdf"
    KROSC_WS_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/artifacts/models")
  if(KROSC_WS_RL_NAV_FOUND)
    target_include_directories(krosc_ws_tests PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
    target_link_libraries(krosc_ws_tests PRIVATE ${ONNXRUNTIME_LIBRARY})
    if(UNIX AND NOT APPLE)
      target_link_options(krosc_ws_tests PRIVATE -Wl,--disable-new-dtags)
    endif()
    target_compile_definitions(krosc_ws_tests PRIVATE KROSC_WS_HAS_ONNXRUNTIME=1)
  endif()
  if(UNIX AND NOT APPLE)
    target_link_libraries(krosc_ws_tests PRIVATE util)
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(krosc_ws_tests PRIVATE -Wno-psabi)
  endif()
  target_compile_options(krosc_ws_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME krosc_ws_tests COMMAND krosc_ws_tests)
endif()
