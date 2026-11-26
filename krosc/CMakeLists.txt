cmake_minimum_required(VERSION 3.25)
project(krosc VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# GCC 15 + libstdc++ variant의 -O2 인라인에서 나오는 오탐 (표준 라이브러리 내부).
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wno-free-nonheap-object -Wno-psabi)
endif()

option(KROSC_BUILD_TESTS "Build krosc tests" ON)
option(KROSC_BUILD_BENCH "Build krosc benchmarks" ON)
option(KROSC_BUILD_VIZ_WEB "Build krosc viz-web gateway" ON)
option(KROSC_BUILD_VIZ "Build krosc viz (Qt6)" ON)
option(KROSC_BUILD_EXAMPLES "Build krosc simulator app" ON)

include(CTest)

add_subdirectory(engine)
if(KROSC_BUILD_VIZ)
  add_subdirectory(viz)
endif()
add_subdirectory(simulator)
if(KROSC_BUILD_VIZ_WEB)
  add_subdirectory(viz-web)
endif()
if(TARGET krosc_bench)
  target_link_libraries(krosc_bench PRIVATE krosc::simulator_assets)
endif()
