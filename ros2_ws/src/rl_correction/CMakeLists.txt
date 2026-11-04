cmake_minimum_required(VERSION 3.16)
project(rl_correction)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
foreach(d rclcpp rcl_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs rl_policy)
  find_package(${d} REQUIRED)
endforeach()

add_executable(corrector src/corrector_node.cpp)   # 실행 파일 이름 = 예전 Python entry point (rl-only.launch.py)
target_include_directories(corrector PRIVATE include)
target_link_libraries(corrector rclcpp::rclcpp rl_policy::rl_policy
  ${rcl_interfaces_TARGETS} ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS} ${std_srvs_TARGETS})
install(TARGETS corrector DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_correct test/test_correct.cpp)
  target_include_directories(test_correct PRIVATE include)
  add_test(NAME test_correct COMMAND test_correct)
endif()

ament_package()
