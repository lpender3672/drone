#pragma once

namespace editor {

// Tiny state holder for whether the ImGui demo panel is rendered. F1
// in the editor's input handling calls `toggle()`, the render path
// reads `is_shown()`. Pure bool wrapper — the value of having a class
// is that future toggles (signal-trace overlay, graph palette) will
// live alongside it as additional members instead of free-floating
// `bool` flags scattered through `main.cpp`.
class DemoToggleState {
public:
    DemoToggleState() = default;
    explicit DemoToggleState(bool initial) : shown_(initial) {}

    bool is_shown() const { return shown_; }

    void toggle()        { shown_ = !shown_; }
    void set(bool value) { shown_ = value; }

private:
    bool shown_ = false;
};

} // namespace editor
