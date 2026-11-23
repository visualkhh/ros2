find_package(Threads REQUIRED)

# 물리/월드/로봇: Qt 없이도 빌드된다 (표준 라이브러리 + core 메시지 타입).
add_library(krosc_sim STATIC
  src/world.cpp
  src/maze_world.cpp
  src/turtlebot3.cpp
  src/runner.cpp
  src/snapshot.cpp
)

add_library(krosc::sim ALIAS krosc_sim)

target_include_directories(krosc_sim
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(krosc_sim PUBLIC krosc::core Threads::Threads)
target_compile_features(krosc_sim PUBLIC cxx_std_20)
target_compile_options(krosc_sim PRIVATE -Wall -Wextra -Wpedantic)

# 진실(월드/로봇) 화면: Qt6. Qt가 없으면 물리 라이브러리만 빌드된다.
find_package(Qt6 6.4 QUIET COMPONENTS Core Gui Widgets)
if(Qt6_FOUND)
  # Homebrew Qt 6.9의 macOS config가 이미 제거된 AGL.framework를 링크하려는 문제 회피.
  if(APPLE AND TARGET WrapOpenGL::WrapOpenGL)
    get_target_property(_krosc_sim_wrap_libs WrapOpenGL::WrapOpenGL INTERFACE_LINK_LIBRARIES)
    if(_krosc_sim_wrap_libs)
      list(FILTER _krosc_sim_wrap_libs EXCLUDE REGEX "AGL")
      set_target_properties(WrapOpenGL::WrapOpenGL PROPERTIES INTERFACE_LINK_LIBRARIES "${_krosc_sim_wrap_libs}")
    endif()
  endif()
  set(CMAKE_AUTOMOC ON)
  add_library(krosc_sim_qt STATIC
    src/view_window.cpp
    include/krosc/sim/view_window.hpp
  )
  add_library(krosc::sim_qt ALIAS krosc_sim_qt)
  target_include_directories(krosc_sim_qt
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(krosc_sim_qt PUBLIC krosc::sim Qt6::Core Qt6::Gui Qt6::Widgets)
  target_compile_options(krosc_sim_qt PRIVATE -Wall -Wextra -Wpedantic)
else()
  message(STATUS "krosc/sim: Qt6를 찾지 못해 진실 화면(SimWindow)은 건너뛴다")
endif()

if(KROSC_BUILD_TESTS)
  add_executable(krosc_sim_tests
    tests/test_sim.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/tests/test_main.cpp
  )
  target_include_directories(krosc_sim_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../core/tests)
  target_link_libraries(krosc_sim_tests PRIVATE krosc::sim)
  target_compile_options(krosc_sim_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME krosc_sim_tests COMMAND krosc_sim_tests)
endif()

install(TARGETS krosc_sim ARCHIVE DESTINATION lib)
if(Qt6_FOUND)
  install(TARGETS krosc_sim_qt ARCHIVE DESTINATION lib)
endif()
install(DIRECTORY include/ DESTINATION include)
