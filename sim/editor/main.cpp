#include <cstdint>
#include <cstdio>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "demo_toggle.hpp"
#include "frame_clock.hpp"

namespace {

void glfw_error_callback(int error, const char* description) {
    std::fprintf(stderr, "glfw error %d: %s\n", error, description);
}

constexpr int   kInitialWidth  = 1280;
constexpr int   kInitialHeight = 720;
constexpr char  kWindowTitle[] = "drone editor";

// GLSL version pinned to 3.30 core — same on macOS / Linux / Windows
// once the GL hints below force a 3.3 core context. The matching
// ImGui_ImplOpenGL3_Init string is "#version 330".
constexpr char  kGlslVersion[] = "#version 330";

// Convert GLFW's monotonic seconds to integer microseconds for FrameClock.
// FrameClock takes uint64_t to share a unit with the block runtime
// (`uint64_t` time_us everywhere); GLFW gives us a double in seconds.
uint64_t now_us() {
    return static_cast<uint64_t>(glfwGetTime() * 1.0e6);
}

} // namespace

int main(int /*argc*/, char** /*argv*/) {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) return 1;

    // 3.3 Core profile, forward-compatible — the minimum macOS will give
    // us in a non-deprecated GL context. Matches ImGui's GL3 backend.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,        GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

    GLFWwindow* window = glfwCreateWindow(
        kInitialWidth, kInitialHeight, kWindowTitle, /*monitor*/ nullptr, /*share*/ nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // vsync — the render loop blocks on the
                          // monitor's refresh rather than spinning at
                          // GPU max. Cheap enough for a tuning UI;
                          // disable later if a slice needs uncapped fps.

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, /*install_callbacks*/ true);
    ImGui_ImplOpenGL3_Init(kGlslVersion);

    editor::FrameClock      frame_clock;
    editor::DemoToggleState demo(true);  // visible at first launch — gives
                                         // a slice-1 user something to look at.

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // F1 toggles the demo panel. Single-shot edge: ImGui's
        // IsKeyPressed default already filters auto-repeat.
        if (ImGui::IsKeyPressed(ImGuiKey_F1, /*repeat*/ false)) {
            demo.toggle();
        }

        frame_clock.tick(now_us());

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Frame-stats overlay — top-left, no decoration. Lives on top
        // of the canvas regardless of what later slices add.
        const ImGuiWindowFlags overlay_flags =
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.35f);
        if (ImGui::Begin("##frame_stats", nullptr, overlay_flags)) {
            ImGui::Text("frame %llu",
                static_cast<unsigned long long>(frame_clock.frame_count()));
            ImGui::Text("dt    %.2f ms", frame_clock.dt_ms());
            ImGui::Text("fps   %.1f",   frame_clock.fps());
            ImGui::Separator();
            ImGui::Text("F1 toggles demo panel");
        }
        ImGui::End();

        if (demo.is_shown()) {
            ImGui::ShowDemoWindow();
        }

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
