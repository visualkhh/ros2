find_package(Threads REQUIRED)

add_library(krosc_core STATIC
  core/context.cpp
  core/executor.cpp
  core/krosc.cpp
  core/log.cpp
  core/net.cpp
  core/node.cpp
  core/parameter.cpp
  core/service.cpp
  core/time.cpp
  core/timer.cpp
  core/topic.cpp
  core/tf.cpp
  core/tf_ros.cpp
  core/urdf.cpp
  core/manipulation/arm_chain.cpp
  core/nodes/cartographer.cpp
  core/nodes/collision_monitor.cpp
  core/nodes/velocity_smoother.cpp
  core/nodes/navigator.cpp
  core/nodes/map_catalog.cpp
  core/nodes/map_io.cpp
  core/nodes/moveit_lite.cpp
  core/nodes/diff_drive_odometry.cpp
  core/nodes/ekf.cpp
  core/nodes/robot_state_publisher.cpp
  core/nodes/tf_echo.cpp
  core/nodes/twist_mux.cpp
)

add_library(krosc::core ALIAS krosc_core)

target_include_directories(krosc_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(krosc_core PUBLIC Threads::Threads)
target_compile_features(krosc_core PUBLIC cxx_std_20)
target_compile_options(krosc_core PRIVATE -Wall -Wextra -Wpedantic)

if(KROSC_BUILD_TESTS)
  add_executable(krosc_core_tests
    tests/test_main.cpp
    tests/test_topics.cpp
    tests/test_timers.cpp
    tests/test_parameters.cpp
    tests/test_tf.cpp
    tests/test_urdf.cpp
    tests/test_nodes.cpp
    tests/test_cartographer.cpp
    tests/test_map_io.cpp
    tests/test_net.cpp
  )
  target_link_libraries(krosc_core_tests PRIVATE krosc::core)
  target_compile_options(krosc_core_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME krosc_core_tests COMMAND krosc_core_tests)
endif()

if(KROSC_BUILD_BENCH)
  add_executable(krosc_bench benchmarks/bench.cpp)
  target_link_libraries(krosc_bench PRIVATE krosc::core)
endif()

install(TARGETS krosc_core ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
