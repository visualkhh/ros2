find_package(Threads REQUIRED)

# 물리/월드 + 런타임: Qt 없이도 빌드된다. 구체 로봇/월드는 assets 라이브러리에 둔다.
add_library(krosc_simulator_core STATIC
  core/world.cpp
  core/runner.cpp
  core/snapshot.cpp
  core/simulator.cpp
)
add_library(krosc::simulator ALIAS krosc_simulator_core)

target_include_directories(krosc_simulator_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/core
)

target_link_libraries(krosc_simulator_core PUBLIC krosc::core Threads::Threads)
target_compile_features(krosc_simulator_core PUBLIC cxx_std_20)
target_compile_options(krosc_simulator_core PRIVATE -Wall -Wextra -Wpedantic)

# 바로 쓸 수 있는 월드/오브젝트/ROS 어댑터. 앱은 필요한 것만 골라 쓰거나
# 자기 월드/로봇을 만들어 Simulator에 직접 넣으면 된다.
add_library(krosc_simulator_assets STATIC
  worlds/maze_world.cpp
  worlds/room_world.cpp
  objects/puck.cpp
  objects/obstacle.cpp
  robots/turtlebot/turtlebot.cpp
)
add_library(krosc::simulator_assets ALIAS krosc_simulator_assets)
target_include_directories(krosc_simulator_assets
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(krosc_simulator_assets PUBLIC krosc::simulator)
target_compile_features(krosc_simulator_assets PUBLIC cxx_std_20)
target_compile_options(krosc_simulator_assets PRIVATE -Wall -Wextra -Wpedantic)

# 진실(월드/로봇) 화면과 시뮬레이터 실행기: Qt6가 필요하다.
find_package(Qt6 6.4 QUIET COMPONENTS Core Gui Widgets)
if(Qt6_FOUND)
  if(APPLE AND TARGET WrapOpenGL::WrapOpenGL)
    get_target_property(_krosc_simulator_wrap_libs WrapOpenGL::WrapOpenGL INTERFACE_LINK_LIBRARIES)
    if(_krosc_simulator_wrap_libs)
      list(FILTER _krosc_simulator_wrap_libs EXCLUDE REGEX "AGL")
      set_target_properties(WrapOpenGL::WrapOpenGL PROPERTIES
        INTERFACE_LINK_LIBRARIES "${_krosc_simulator_wrap_libs}")
    endif()
  endif()
  set(CMAKE_AUTOMOC ON)
  add_library(krosc_simulator_qt STATIC
    core/simulator_window.cpp
    include/krosc/simulator/simulator_window.hpp
  )
  add_library(krosc::simulator_qt ALIAS krosc_simulator_qt)
  target_include_directories(krosc_simulator_qt
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(krosc_simulator_qt PUBLIC
    krosc::simulator Qt6::Core Qt6::Gui Qt6::Widgets)
  target_compile_options(krosc_simulator_qt PRIVATE -Wall -Wextra -Wpedantic)

  if(KROSC_BUILD_EXAMPLES AND TARGET krosc::viz)
    add_executable(krosc_simulator
      apps/demo/main.cpp
      apps/demo/robot_bridge.cpp
      apps/demo/map_publisher.cpp
    )
    target_include_directories(krosc_simulator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/apps/demo)
    target_link_libraries(krosc_simulator PRIVATE
      krosc::simulator_assets
      krosc::simulator_qt
      krosc::viz
    )
    target_compile_definitions(krosc_simulator PRIVATE
      KROSC_SIMULATOR_URDF="${CMAKE_CURRENT_SOURCE_DIR}/robots/turtlebot/turtlebot.urdf"
    )
    target_compile_options(krosc_simulator_core PRIVATE -Wall -Wextra -Wpedantic)
  endif()
else()
  message(STATUS "krosc/simulator: Qt6를 찾지 못해 SimulatorWindow/앱은 건너뛴다")
endif()

if(KROSC_BUILD_TESTS)
  add_executable(krosc_simulator_tests
    tests/test_sim.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../engine/tests/test_main.cpp
  )
  target_include_directories(krosc_simulator_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../engine/tests)
  target_link_libraries(krosc_simulator_tests PRIVATE krosc::simulator_assets)
  target_compile_definitions(krosc_simulator_tests PRIVATE
    KROSC_SIMULATOR_URDF="${CMAKE_CURRENT_SOURCE_DIR}/robots/turtlebot/turtlebot.urdf"
  )
  target_compile_options(krosc_simulator_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME krosc_simulator_tests COMMAND krosc_simulator_tests)
endif()

install(TARGETS krosc_simulator_core krosc_simulator_assets ARCHIVE DESTINATION lib)
if(Qt6_FOUND)
  install(TARGETS krosc_simulator_qt ARCHIVE DESTINATION lib)
endif()
install(DIRECTORY include/ DESTINATION include)
