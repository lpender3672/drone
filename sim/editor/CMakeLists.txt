# Editor target — Tier 3 first slice. Brings up an ImGui window that
# (in this slice) just shows the ImGui demo panel + a frame-stats overlay.
# Future slices add the graph canvas (ImNodes), live signal plots
# (ImPlot), and a palette wired to the BlockFactory.
#
# Stack: GLFW (window + input) + OpenGL 3.3 core (renderer) + ImGui.
# All deps via FetchContent so a clean clone of the repo builds the
# editor without per-machine setup, matching the existing nlohmann_json /
# Eigen pattern.

include(FetchContent)

# ---- GLFW ----------------------------------------------------------------
# Prefer system install (homebrew / apt), fall back to FetchContent.
find_package(glfw3 3.3 QUIET)
if(NOT TARGET glfw)
    set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        3.4
    )
    FetchContent_MakeAvailable(glfw)
endif()

# ---- OpenGL --------------------------------------------------------------
# macOS exposes GL via a framework; Linux via libGL; Windows via opengl32.
# CMake's FindOpenGL handles all three.
find_package(OpenGL REQUIRED)

# ---- Dear ImGui (static library compiled from the FetchContent tree) ----
# Header-only is a misnomer for ImGui — there are a handful of .cpp's
# (core + the GLFW + OpenGL3 backends) that need to be compiled into
# the editor binary.
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.4
)
FetchContent_MakeAvailable(imgui)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)
# Quiet the warnings from ImGui's own translation units — they're not ours
# to police, and `-Werror`-clean compiles for the editor's own code still
# benefit from -Wall/-Wextra.
target_compile_options(imgui PRIVATE -w)

# ---- Editor executable ---------------------------------------------------
add_executable(editor
    main.cpp
)
target_include_directories(editor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
# sim_lib is the host-side block runtime + concrete block headers. Linking
# it now (even though slice 1 doesn't use it) means slice 2's "load
# quad.json into the canvas" already has the types in scope.
target_link_libraries(editor PRIVATE sim_lib imgui glfw OpenGL::GL)
