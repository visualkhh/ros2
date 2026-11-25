find_package(Threads REQUIRED)

add_library(krosc_core STATIC
  core/context.cpp
  core/executor.cpp
  core/krosc.cpp
  core/log/log.cpp
  core/net/net.cpp
  core/net/net_protocol.cpp
  core/net/net_peer.cpp
  core/net/rpc.cpp
  core/msgs/rpc_builtin.cpp
  core/msgs/codec_basic.cpp
  core/msgs/codec_nav2.cpp
  core/msgs/codec_parameters.cpp
  core/msgs/codec_ros2_control.cpp
  core/msgs/codec_actions.cpp
  core/node.cpp
  core/parameter.cpp
  core/service.cpp
  core/time/time.cpp
  core/time/timer.cpp
  core/topic.cpp
  core/tf/tf.cpp
  core/tf/tf_ros.cpp
  core/urdf.cpp
  core/manipulation/arm_chain.cpp
  core/nodes/cartographer.cpp
  core/nodes/collision_monitor.cpp
  core/nodes/velocity_smoother.cpp
  core/nodes/navigator.cpp
  core/nodes/map_catalog.cpp
  core/nodes/map_saver.cpp
  core/nodes/map_server.cpp
  core/nodes/map_io.cpp
  core/nodes/diff_drive_odometry.cpp
  core/nodes/ekf.cpp
  core/nodes/robot_state_publisher.cpp
  core/nodes/tf_echo.cpp
  core/nodes/twist_mux.cpp
  core/nodes/nav2/behavior_server.cpp
  core/nodes/nav2/bt_navigator.cpp
  core/nodes/nav2/controllers/controller_plugins.cpp
  core/nodes/nav2/controller_server.cpp
  core/nodes/nav2/controllers/dwb_local_planner.cpp
  core/nodes/nav2/controllers/mppi_controller.cpp
  core/nodes/nav2/controllers/regulated_pure_pursuit_controller.cpp
  core/nodes/nav2/controllers/rotation_shim_controller.cpp
  core/nodes/nav2/costmap/costmap.cpp
  core/nodes/nav2/costmap/costmap_2d_ros.cpp
  core/nodes/nav2/planners/navfn_planner.cpp
  core/nodes/nav2/planners/planner_plugins.cpp
  core/nodes/nav2/planner_server.cpp
  core/nodes/nav2/planners/smac_planner_2d.cpp
  core/nodes/nav2/planners/smac_planner_hybrid.cpp
  core/nodes/nav2/planners/theta_star_planner.cpp
  core/ros2_control/controller_manager.cpp
  core/ros2_control/diff_drive_controller.cpp
  core/ros2_control/forward_command_controller.cpp
  core/ros2_control/joint_state_broadcaster.cpp
  core/ros2_control/joint_trajectory_controller.cpp
  core/nodes/moveit/kinematics/arm_chain_kinematics.cpp
  core/nodes/moveit/planners/cartesian_planner.cpp
  core/nodes/moveit/controllers/controller_manager_plugins.cpp
  core/nodes/moveit/planners/joint_space_planner.cpp
  core/nodes/moveit/kinematics/kinematics_plugins.cpp
  core/nodes/moveit/move_group.cpp
  core/nodes/moveit/planners/planner_manager.cpp
  core/nodes/moveit/planners/planner_plugins.cpp
  core/nodes/moveit/controllers/simple_controller_manager.cpp
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
    tests/test_map_nodes.cpp
    tests/test_net.cpp
    tests/test_nav2.cpp
    tests/test_ros2_control.cpp
  )
  target_link_libraries(krosc_core_tests PRIVATE krosc::core)
  target_compile_options(krosc_core_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME krosc_core_tests COMMAND krosc_core_tests)
endif()

if(KROSC_BUILD_BENCH)
  add_executable(krosc_bench benchmarks/bench.cpp)
  target_link_libraries(krosc_bench PRIVATE krosc::core)
  add_executable(krosc_nav2_bench benchmarks/nav2_bench.cpp)
  target_link_libraries(krosc_nav2_bench PRIVATE krosc::core)
endif()

install(TARGETS krosc_core ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
