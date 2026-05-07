cmake_minimum_required(VERSION 3.14)
include(FetchContent)
project(drone_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Eigen: prefer system install (CI), fall back to a GitLab clone via
# FetchContent. The repo also has an `eigen/` submodule used by the
# embedded targets (ekf/CMakeLists.txt, ekf-teensy/platformio.ini); the
# host build no longer relies on that path so `git submodule update --init`
# isn't required just to build sim or tests.
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
  )
  FetchContent_MakeAvailable(eigen)
endif()

# nlohmann/json — header-only JSON parser used by the Tier 2 graph loader.
# Prefer a system install; fall back to FetchContent. Same flow as the test
# build (tests/CMakeLists.txt).
find_package(nlohmann_json 3.11 QUIET)
if(NOT TARGET nlohmann_json::nlohmann_json)
    FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.3
    )
    set(JSON_BuildTests OFF CACHE INTERNAL "")
    FetchContent_MakeAvailable(json)
endif()

# Warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# MinGW on Windows: debug builds with heavy template instantiation exceed the
# default COFF section limit. -Wa,-mbig-obj raises it to 2^31 sections.
if(WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-Wa,-mbig-obj)
endif()

# Header-only library (interface library for header-only code)
add_library(sim_lib INTERFACE)

target_include_directories(sim_lib INTERFACE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/../shared
    ${CMAKE_SOURCE_DIR}/../shared/sensors
    ${CMAKE_SOURCE_DIR}/../ekf/shared
)

target_link_libraries(sim_lib INTERFACE Eigen3::Eigen nlohmann_json::nlohmann_json)

# Main executable
add_executable(quad_sim
    src/main.cpp
    ${CMAKE_SOURCE_DIR}/../ekf/shared/ekf16.cpp
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

# Tests live in the root-level tests/ tree alongside shared/ tests, not
# inside sim/. Use `cmake --preset debug -S tests` to configure them.
