cmake_minimum_required(VERSION 3.10)
project(sense_hat_ins)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to build tests
option(BUILD_TESTING "Build unit tests" OFF)

# Find Eigen3 (Linear Algebra Library)
find_package(Eigen3 3.3 REQUIRED)

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

# Main executable (only for RPI/Linux with RTIMULib)
if(RPI_BUILD)
    add_executable(ins_runner main.cpp)
    target_link_libraries(ins_runner RTIMULib Eigen3::Eigen)
endif()

# Windows mock executable (for compilation testing)
if(WINDOWS_BUILD)
    add_executable(ins_runner_mock main_mock.cpp)
    target_link_libraries(ins_runner_mock Eigen3::Eigen)
endif()

# Testing configuration
if(BUILD_TESTING)
    enable_testing()
    find_package(GTest REQUIRED)
    
    # Create test executable
    add_executable(eskf_tests 
        tests/test_eskf_simple.cpp
    )
    
    # Link libraries
    target_link_libraries(eskf_tests 
        GTest::gtest 
        GTest::gtest_main
        Eigen3::Eigen
    )
    
    # For MSYS2/MinGW, add specific compiler flags
    if(WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(eskf_tests PRIVATE -DEIGEN_NO_DEBUG)
        target_link_options(eskf_tests PRIVATE -static-libgcc -static-libstdc++)
    endif()
    
    target_include_directories(eskf_tests PRIVATE .)
    
    # Enable Google Test discovery for VS Code Test Explorer
    include(GoogleTest)
    gtest_discover_tests(eskf_tests 
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    
    # Add custom test target
    add_custom_target(run_tests
        COMMAND ${CMAKE_CTEST_COMMAND} --verbose
        DEPENDS eskf_tests
        COMMENT "Running all tests"
    )
endif()
