cmake_minimum_required(VERSION 3.16)
project(rl_policy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(onnxruntime_vendor REQUIRED)   # sudo apt install ros-lyrical-onnxruntime-vendor → onnxruntime::onnxruntime

add_library(rl_policy SHARED src/onnx_policy.cpp)
target_include_directories(rl_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(rl_policy PUBLIC onnxruntime::onnxruntime)

install(TARGETS rl_policy EXPORT export_rl_policy ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  add_executable(test_onnx_policy test/test_onnx_policy.cpp)
  target_link_libraries(test_onnx_policy rl_policy)
  add_test(NAME test_onnx_policy COMMAND test_onnx_policy ${CMAKE_CURRENT_SOURCE_DIR}/test/data/tiny.onnx)
endif()

ament_export_targets(export_rl_policy HAS_LIBRARY_TARGET)
ament_export_dependencies(onnxruntime_vendor)
ament_package()
