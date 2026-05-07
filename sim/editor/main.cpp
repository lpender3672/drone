#include <cstdint>
#include <cstdio>
#include <string>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imnodes.h>
#include <nfd.h>

#include <core/block_factory.hpp>
#include <quadcopter/registrations.hpp>

#include "app_state.hpp"
#include "canvas_view.hpp"
#include "demo_toggle.hpp"
#include "frame_clock.hpp"
#include "run_state.hpp"

namespace {

void glfw_error_callback(int error, const char* description) {
    std::fprintf(stderr, "glfw error %d: %s\n", error, description);
}

constexpr int   kInitialWidth  = 1280;
constexpr int   kInitialHeight = 720;
constexpr char  kWindowTitle[] = "drone editor";
constexpr char  kGlslVersion[] = "#version 330";

// Fixed-height status bar pinned to the bottom of the viewport.
constexpr float kStatusBarHeight = 24.0f;

// Toolbar height (Run / Pause / Reset row) sits under the menu bar.
constexpr float kToolbarHeight   = 32.0f;

uint64_t now_us() {
    return static_cast<uint64_t>(glfwGetTime() * 1.0e6);
}

// Open native file dialog; on accept, returns the chosen path. On
// cancel or error, returns empty string. Filters to JSON; user can
// drop the filter to see other files if they really want.
std::string pick_graph_file() {
    nfdu8filteritem_t filters[] = { { "Graph spec", "json" } };
    nfdu8char_t* out = nullptr;
    nfdopendialogu8args_t args{};
    args.filterList  = filters;
    args.filterCount = 1;
    const nfdresult_t r = NFD_OpenDialogU8_With(&out, &args);
    if (r != NFD_OKAY || out == nullptr) return {};
    std::string path = out;
    NFD_FreePathU8(out);
    return path;
}

// Save dialog — symmetric to pick_graph_file. Returns chosen path or
// empty on cancel.
std::string pick_save_path() {
    nfdu8filteritem_t filters[] = { { "Graph spec", "json" } };
    nfdu8char_t* out = nullptr;
    nfdsavedialogu8args_t args{};
    args.filterList     = filters;
    args.filterCount    = 1;
    args.defaultName    = "untitled.json";
    const nfdresult_t r = NFD_SaveDialogU8_With(&out, &args);
    if (r != NFD_OKAY || out == nullptr) return {};
    std::string path = out;
    NFD_FreePathU8(out);
    return path;
}

// Map StatusLevel to an ImGui colour. Slice 3 colours: white-ish for
// info, amber for warning, red for error.
ImVec4 colour_for(editor::StatusLevel level) {
    switch (level) {
        case editor::StatusLevel::Info:    return ImVec4(0.85f, 0.85f, 0.85f, 1.0f);
        case editor::StatusLevel::Warning: return ImVec4(1.00f, 0.75f, 0.25f, 1.0f);
        case editor::StatusLevel::Error:   return ImVec4(1.00f, 0.40f, 0.40f, 1.0f);
    }
    return ImVec4(1, 1, 1, 1);
}

} // namespace

int main(int /*argc*/, char** /*argv*/) {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) return 1;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,        GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

    GLFWwindow* window = glfwCreateWindow(
        kInitialWidth, kInitialHeight, kWindowTitle, nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImNodes::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(kGlslVersion);
    NFD_Init();

    editor::FrameClock      frame_clock;
    editor::DemoToggleState demo(false);
    editor::CanvasView      canvas;
    editor::RunState        run;

    shared::BlockFactory factory;
    sim::register_quadrotor_blocks(factory);
    editor::AppState app(std::move(factory));

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (ImGui::IsKeyPressed(ImGuiKey_F1, false)) demo.toggle();
        frame_clock.tick(now_us());

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ---- Menu bar ----------------------------------------------------
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Open...", "Ctrl+O")) {
                    const std::string path = pick_graph_file();
                    if (!path.empty() && app.load_graph(path, now_us())) {
                        canvas.set_graph(&app.graph_mut());
                    }
                }
                if (ImGui::MenuItem("Reload", "Ctrl+R",
                                    false, !app.current_path().empty())) {
                    if (app.reload(now_us())) {
                        canvas.set_graph(&app.graph_mut());
                    }
                }
                ImGui::Separator();
                const bool has_graph = app.graph().block_count() > 0;
                if (ImGui::MenuItem("Save", "Ctrl+S", false,
                                    has_graph && !app.current_path().empty())) {
                    app.save_graph(app.current_path(), now_us());
                }
                if (ImGui::MenuItem("Save As...", "Shift+Ctrl+S", false,
                                    has_graph)) {
                    const std::string path = pick_save_path();
                    if (!path.empty()) {
                        app.save_graph(path, now_us());
                    }
                }
                if (ImGui::BeginMenu("Recent",
                                     !app.recent_files().entries().empty())) {
                    for (const auto& p : app.recent_files().entries()) {
                        if (ImGui::MenuItem(p.c_str())) {
                            if (app.load_graph(p, now_us())) {
                                canvas.set_graph(&app.graph_mut());
                            }
                        }
                    }
                    ImGui::EndMenu();
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Quit")) {
                    glfwSetWindowShouldClose(window, GLFW_TRUE);
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("View")) {
                if (ImGui::MenuItem("ImGui demo panel", "F1", demo.is_shown())) {
                    demo.toggle();
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        // ---- Toolbar (Run / Pause / Reset) -------------------------------
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y));
        ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, kToolbarHeight));
        const ImGuiWindowFlags toolbar_flags =
            ImGuiWindowFlags_NoDecoration   | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
        if (ImGui::Begin("##toolbar", nullptr, toolbar_flags)) {
            const bool can_run   = !run.is_running();
            const bool can_pause =  run.is_running();
            const bool has_graph = app.graph().block_count() > 0;

            ImGui::BeginDisabled(!can_run || !has_graph);
            if (ImGui::Button("Run")) {
                run.run();
                // Slice 6 will hook this to the actual tick loop. Until
                // then, the button just flips the state machine + posts.
                app.status_bar_mut()
                    .post(editor::StatusLevel::Info,
                          "run requested (sim wiring lands in slice 6)",
                          now_us());
            }
            ImGui::EndDisabled();

            ImGui::SameLine();
            ImGui::BeginDisabled(!can_pause);
            if (ImGui::Button("Pause")) {
                run.pause();
                app.status_bar_mut()
                    .post(editor::StatusLevel::Info,
                          "paused", now_us());
            }
            ImGui::EndDisabled();

            ImGui::SameLine();
            if (ImGui::Button("Reset")) {
                run.reset();
                app.status_bar_mut()
                    .post(editor::StatusLevel::Info,
                          "reset", now_us());
            }

            ImGui::SameLine();
            const char* phase_label = run.is_running()  ? "running"
                                    : run.is_paused()   ? "paused"
                                    :                     "stopped";
            ImGui::Text("  state: %s", phase_label);
        }
        ImGui::End();

        // ---- Canvas (fills space between toolbar and status bar) ---------
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x,
                                       viewport->WorkPos.y + kToolbarHeight));
        ImGui::SetNextWindowSize(ImVec2(
            viewport->WorkSize.x,
            viewport->WorkSize.y - kToolbarHeight - kStatusBarHeight));
        const ImGuiWindowFlags canvas_flags =
            ImGuiWindowFlags_NoDecoration   | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus |
            ImGuiWindowFlags_NoNavFocus     | ImGuiWindowFlags_NoSavedSettings;
        if (ImGui::Begin("##canvas", nullptr, canvas_flags)) {
            canvas.render();
        }
        ImGui::End();

        // ---- Frame stats overlay (top-left, on top of toolbar) -----------
        const ImGuiWindowFlags overlay_flags =
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
        ImGui::SetNextWindowPos(
            ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 130,
                   viewport->WorkPos.y + 4),
            ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.55f);
        if (ImGui::Begin("##frame_stats", nullptr, overlay_flags)) {
            ImGui::Text("%6.1f fps", frame_clock.fps());
            ImGui::Text("%6.2f ms",  frame_clock.dt_ms());
        }
        ImGui::End();

        // ---- Status bar (bottom of viewport) -----------------------------
        ImGui::SetNextWindowPos(ImVec2(
            viewport->WorkPos.x,
            viewport->WorkPos.y + viewport->WorkSize.y - kStatusBarHeight));
        ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, kStatusBarHeight));
        const ImGuiWindowFlags statusbar_flags =
            ImGuiWindowFlags_NoDecoration   | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
        if (ImGui::Begin("##statusbar", nullptr, statusbar_flags)) {
            const auto msg = app.status_bar().current(now_us());
            if (msg) {
                ImGui::TextColored(colour_for(msg->level), "%s",
                                   msg->message.c_str());
            } else if (!app.current_path().empty()) {
                ImGui::TextDisabled("%s", app.current_path().c_str());
            } else {
                ImGui::TextDisabled("no graph loaded");
            }
        }
        ImGui::End();

        if (demo.is_shown()) ImGui::ShowDemoWindow();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    NFD_Quit();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImNodes::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
