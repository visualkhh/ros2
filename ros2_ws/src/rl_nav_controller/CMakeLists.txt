cmake_minimum_required(VERSION 3.16)
project(rl_nav_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
set(deps rclcpp rclcpp_lifecycle nav2_core nav2_util nav2_costmap_2d nav2_ros_common pluginlib
  geometry_msgs nav_msgs sensor_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs rl_policy)
foreach(d ${deps})
  find_package(${d} REQUIRED)
endforeach()

add_library(rl_nav_controller SHARED src/rl_nav_controller.cpp)
target_include_directories(rl_nav_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(rl_nav_controller PUBLIC
  rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle nav2_core::nav2_core nav2_util::nav2_util_core
  nav2_costmap_2d::nav2_costmap_2d_core nav2_ros_common::nav2_ros_common pluginlib::pluginlib
  tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs rl_policy::rl_policy
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_srvs_TARGETS})

pluginlib_export_plugin_description_file(nav2_core rl_nav_controller.xml)

install(TARGETS rl_nav_controller ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(rl_nav_controller)
ament_export_dependencies(${deps})
ament_package()
