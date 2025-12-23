cmake_minimum_required(VERSION 3.10)
project(sense_hat_ins)

# C++ Standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_TESTING "Build unit tests" OFF)

# Find Dependencies
# Use Eigen3 source directly from submodule
include_directories(${CMAKE_SOURCE_DIR}/../eigen)

# Platform Detection
if(WIN32)
    set(WINDOWS_BUILD ON)
    message(STATUS "Building for Windows")
else()
    set(RPI_BUILD ON)
    message(STATUS "Building for Raspberry Pi/Linux")
endif()

# Include Directories
include_directories(${CMAKE_SOURCE_DIR}/shared)

# Raspberry Pi/Linux Executables
if(RPI_BUILD)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1")

    add_executable(ins_runner rpi/main.cpp rpi/gps_interface.cpp shared/eskf.cpp)
    target_link_libraries(ins_runner RTIMULib)

    add_executable(sense_hat_logger rpi/sense_hat_logger.cpp shared/eskf.cpp)
    target_link_libraries(sense_hat_logger RTIMULib)

    add_executable(gps_logger rpi/gps_logger.cpp rpi/gps_interface.cpp)
endif()

# Testing
if(BUILD_TESTING)
    include(CTest)
    find_package(GTest REQUIRED)
    include(GoogleTest)
    add_executable(eskf_tests tests/test_eskf.cpp shared/eskf.cpp)
    target_link_libraries(eskf_tests GTest::gtest GTest::gtest_main)
    target_include_directories(eskf_tests PRIVATE ${CMAKE_SOURCE_DIR}/shared)
    gtest_discover_tests(eskf_tests)
endif()
