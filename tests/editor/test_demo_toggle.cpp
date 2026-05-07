#include <gtest/gtest.h>

#include <demo_toggle.hpp>

using editor::DemoToggleState;

// DemoToggleState gates ImGui::ShowDemoWindow() in the editor's render
// loop. Pure bool wrapper; F1 in main.cpp calls toggle(), the renderer
// reads is_shown(). Trivial, but it's the first piece of editor *state*
// so it lands here as the TDD pattern for richer state to come.

TEST(DemoToggleState, DefaultsToHidden) {
    DemoToggleState s;
    EXPECT_FALSE(s.is_shown());
}

TEST(DemoToggleState, ConstructWithExplicitInitial) {
    DemoToggleState shown(true);
    EXPECT_TRUE(shown.is_shown());

    DemoToggleState hidden(false);
    EXPECT_FALSE(hidden.is_shown());
}

TEST(DemoToggleState, ToggleFlipsState) {
    DemoToggleState s;
    s.toggle();
    EXPECT_TRUE(s.is_shown());
    s.toggle();
    EXPECT_FALSE(s.is_shown());
}

TEST(DemoToggleState, SetExplicitlyOverridesCurrent) {
    DemoToggleState s;
    s.set(true);
    EXPECT_TRUE(s.is_shown());
    s.set(true);
    EXPECT_TRUE(s.is_shown());  // idempotent
    s.set(false);
    EXPECT_FALSE(s.is_shown());
}
