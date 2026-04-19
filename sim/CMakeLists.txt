cmake_minimum_required(VERSION 3.14)
include(FetchContent)
project(drone_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Eigen: prefer system install (CI), fall back to local source tree
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
  FetchContent_Declare(eigen SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../eigen)
  FetchContent_MakeAvailable(eigen)
endif()

# Warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Header-only library (interface library for header-only code)
add_library(sim_lib INTERFACE)

target_include_directories(sim_lib INTERFACE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/../shared
    ${CMAKE_SOURCE_DIR}/../shared/sensors
    ${CMAKE_SOURCE_DIR}/../ekf/shared
)

target_link_libraries(sim_lib INTERFACE Eigen3::Eigen)

# Main executable
add_executable(quad_sim
    src/main.cpp
    ${CMAKE_SOURCE_DIR}/../ekf/shared/ekf16d.cpp
)

target_link_libraries(quad_sim PRIVATE sim_lib)

# Optional: Release build optimizations
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(quad_sim PRIVATE -O3 -march=native)
endif()

# Print configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")

# Testing option
option(SIM_BUILD_TESTS "Build tests" OFF)

if(SIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
