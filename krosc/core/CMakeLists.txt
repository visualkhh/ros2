find_package(Threads REQUIRED)

add_library(krosc_core STATIC
  src/context.cpp
  src/executor.cpp
  src/krosc.cpp
  src/log.cpp
  src/net.cpp
  src/node.cpp
  src/parameter.cpp
  src/service.cpp
  src/time.cpp
  src/timer.cpp
  src/topic.cpp
  src/tf.cpp
  src/tf_ros.cpp
  src/urdf.cpp
  src/nodes/cartographer.cpp
  src/nodes/diff_drive_odometry.cpp
  src/nodes/ekf.cpp
  src/nodes/robot_state_publisher.cpp
  src/nodes/tf_echo.cpp
  src/nodes/twist_mux.cpp
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
