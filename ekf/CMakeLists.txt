cmake_minimum_required(VERSION 3.10)
project(sense_hat_ins)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to build tests
option(BUILD_TESTING "Build unit tests" OFF)

# Find Eigen3 (Linear Algebra Library)
find_package(Eigen3 CONFIG REQUIRED)

# Platform-specific configurations
if(WIN32)
    # Windows-specific settings
    set(WINDOWS_BUILD ON)
    message(STATUS "Building for Windows")
else()
    # Raspberry Pi/Linux settings
    set(RPI_BUILD ON)
    message(STATUS "Building for Raspberry Pi/Linux")
endif()

# Include Eigen headers
include_directories(${EIGEN3_INCLUDE_DIR})

# Include shared headers (eskf.h, sensor_io.h, etc.)
include_directories(${CMAKE_SOURCE_DIR}/shared)

# Main executable (only for RPI/Linux with RTIMULib)
if(RPI_BUILD)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1")

    add_executable(ins_runner rpi/main.cpp rpi/gps_interface.cpp shared/eskf.cpp)
    target_link_libraries(ins_runner RTIMULib Eigen3::Eigen)

    # Sense Hat Logger executable
    add_executable(sense_hat_logger rpi/sense_hat_logger.cpp shared/eskf.cpp)
    target_link_libraries(sense_hat_logger RTIMULib Eigen3::Eigen)

    # GPS Logger executable
    add_executable(gps_logger rpi/gps_logger.cpp rpi/gps_interface.cpp)
    target_link_libraries(gps_logger)
endif()

# Testing configuration
if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    set(TEST_SOURCES
        tests/test_eskf.cpp
        shared/eskf.cpp
    )

    # Create test executable
    add_executable(eskf_tests 
        ${TEST_SOURCES}
    )
    
    # Link libraries
    target_link_libraries(eskf_tests 
        GTest::gtest 
        GTest::gtest_main
        Eigen3::Eigen
    )
    
    target_include_directories(eskf_tests PRIVATE ${CMAKE_SOURCE_DIR}/shared)
endif()
