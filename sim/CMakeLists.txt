cmake_minimum_required(VERSION 3.14)
include(FetchContent)
project(drone_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Eigen
FetchContent_Declare(
  eigen
  SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../eigen
)
FetchContent_MakeAvailable(eigen)

# ImGui
add_library(imgui STATIC
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/backends/imgui_impl_glfw.cpp
    imgui/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC imgui imgui/backends)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

# ImPlot
add_library(implot STATIC
    implot/implot.cpp
    implot/implot_items.cpp
)
target_include_directories(implot PUBLIC implot)
target_link_libraries(implot PUBLIC imgui)

# Warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Header-only library (interface library for header-only code)
add_library(sim_lib INTERFACE)

target_include_directories(sim_lib INTERFACE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/../shared  # For shared sensor headers
)

target_link_libraries(sim_lib INTERFACE Eigen3::Eigen)

# Main executable
add_executable(quad_sim
    src/main.cpp
)

target_link_libraries(quad_sim PRIVATE sim_lib implot)

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
