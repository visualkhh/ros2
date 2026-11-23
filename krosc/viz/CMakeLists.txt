find_package(Qt6 6.4 QUIET COMPONENTS Core Gui Widgets)

if(NOT Qt6_FOUND)
  message(STATUS "krosc/viz: Qt6를 찾지 못해 viz 패키지를 건너뛴다 (qt6-base-dev 설치 필요)")
  return()
endif()

# Homebrew Qt 6.9의 macOS config는 이미 제거된 AGL.framework를 링크하려 한다.
# 실제 Qt6는 AGL을 쓰지 않으므로 링크 목록에서만 빼 준다 (Ubuntu/Docker에는 해당 없음).
if(APPLE AND TARGET WrapOpenGL::WrapOpenGL)
  get_target_property(_krosc_wrap_opengl_libs WrapOpenGL::WrapOpenGL INTERFACE_LINK_LIBRARIES)
  if(_krosc_wrap_opengl_libs)
    list(FILTER _krosc_wrap_opengl_libs EXCLUDE REGEX "AGL")
    set_target_properties(WrapOpenGL::WrapOpenGL PROPERTIES INTERFACE_LINK_LIBRARIES "${_krosc_wrap_opengl_libs}")
  endif()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_library(krosc_viz STATIC
  src/viz_model.cpp
  src/viz_window.cpp
  src/scene_view.cpp
  include/krosc/viz/viz_model.hpp
  include/krosc/viz/viz_window.hpp
  include/krosc/viz/scene_view.hpp
)

add_library(krosc::viz ALIAS krosc_viz)

target_include_directories(krosc_viz
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(krosc_viz
  PUBLIC
    krosc::core
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)

target_compile_options(krosc_viz PRIVATE -Wall -Wextra -Wpedantic)

# viz 패키지 단독 실행기: 빈 그리드와 패널을 띄워 viz 자체를 시험한다.
add_executable(krosc_viz_app src/main.cpp)
target_link_libraries(krosc_viz_app PRIVATE krosc::viz)

install(TARGETS krosc_viz ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
