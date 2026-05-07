cmake_minimum_required(VERSION 3.14)
include(FetchContent)
project(drone_tests VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Eigen — prefer system install (CI), fall back to a GitLab clone via
# FetchContent. (The embedded targets still use the `eigen/` submodule;
# the host test build no longer depends on it.)
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
  )
  FetchContent_MakeAvailable(eigen)
endif()

# GoogleTest — prefer system install (libgtest-dev on CI) so we don't pay
# the ~10 s FetchContent clone on every CI run. Falls back to FetchContent
# for environments without a packaged gtest (e.g. local Windows/MSYS2).
find_package(GTest QUIET)
if(NOT TARGET GTest::gtest)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

include(GoogleTest)

# nlohmann/json — JSON parser used by the graph loader. Single header,
# header-only, MIT, no transitive deps. Prefer system install if present;
# FetchContent otherwise.
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

# Warnings.
add_compile_options(-Wall -Wextra -Wpedantic)

# MinGW debug builds with heavy template instantiation can exceed the COFF
# section limit. -Wa,-mbig-obj raises it. Same flag the sim build uses.
if(WIN32 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-Wa,-mbig-obj)
endif()

# Include paths shared by every test target — covers the shared/ block runtime,
# sensor types, sim's wrapper headers, the EKF math headers, and reusable
# test mocks (tests/mocks/).
add_library(test_includes INTERFACE)
target_include_directories(test_includes INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/../shared
    ${CMAKE_CURRENT_LIST_DIR}/../shared/sensors
    ${CMAKE_CURRENT_LIST_DIR}/../sim/src
    ${CMAKE_CURRENT_LIST_DIR}/../sim/editor
    ${CMAKE_CURRENT_LIST_DIR}/../ekf/shared
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(test_includes INTERFACE Eigen3::Eigen nlohmann_json::nlohmann_json)

# Hand the repo root in as a compile definition so tests that load
# JSON specs (round-trip, etc.) don't depend on CWD.
target_compile_definitions(test_includes INTERFACE
    DRONE_REPO_ROOT="${CMAKE_CURRENT_LIST_DIR}/.."
)

# Single test binary for fast feedback. If it grows slow, split by domain
# (shared_tests, sim_tests) — never re-establish a separate runner per
# subtree.
file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/sim/test_*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/shared/test_*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/shared/**/test_*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/editor/test_*.cpp"
)

# Round-trip test instantiates EKF16d via the block factory, so the
# EKF translation units join the test binary. (The quadrotor tests
# elsewhere don't touch the factory and don't need these.)
add_executable(drone_tests
    ${TEST_SOURCES}
    ${CMAKE_CURRENT_LIST_DIR}/../ekf/shared/ekf16.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../ekf/shared/ekf16d.cpp
)

target_link_libraries(drone_tests
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        test_includes
)

enable_testing()
gtest_discover_tests(drone_tests
    DISCOVERY_MODE PRE_TEST
)

message(STATUS "Test binary will include: ${TEST_SOURCES}")
