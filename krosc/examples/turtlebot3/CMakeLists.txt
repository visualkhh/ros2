add_executable(krosc_turtlebot3
  main.cpp
  map_publisher.cpp
  robot_bridge.cpp
)

target_link_libraries(krosc_turtlebot3
  PRIVATE
    krosc::viz
    krosc::sim_qt
)

target_compile_definitions(krosc_turtlebot3 PRIVATE
  KROSC_TURTLEBOT3_URDF="${CMAKE_CURRENT_SOURCE_DIR}/description/turtlebot3_waffle_pi.urdf"
)

target_compile_options(krosc_turtlebot3 PRIVATE -Wall -Wextra -Wpedantic)
