#include <gtest/gtest.h>

#include <run_state.hpp>

using editor::RunState;
using editor::RunPhase;

// RunState is the editor's Run / Pause / Stop state machine. Slice 3
// stubs the actual sim ticking — the buttons just flip this enum and
// post a status. Slice 6 will hook the tick loop to current() to drive
// blocks per frame.

TEST(RunState, FreshIsStopped) {
    RunState s;
    EXPECT_EQ(s.current(), RunPhase::Stopped);
    EXPECT_FALSE(s.is_running());
    EXPECT_FALSE(s.is_paused());
}

TEST(RunState, RunFromStoppedTransitionsToRunning) {
    RunState s;
    s.run();
    EXPECT_EQ(s.current(), RunPhase::Running);
    EXPECT_TRUE(s.is_running());
}

TEST(RunState, PauseFromRunningTransitionsToPaused) {
    RunState s;
    s.run();
    s.pause();
    EXPECT_EQ(s.current(), RunPhase::Paused);
    EXPECT_TRUE(s.is_paused());
    EXPECT_FALSE(s.is_running());
}

TEST(RunState, RunFromPausedResumes) {
    RunState s;
    s.run();
    s.pause();
    s.run();
    EXPECT_EQ(s.current(), RunPhase::Running);
}

TEST(RunState, ResetFromAnyStateReturnsToStopped) {
    RunState a; a.reset();
    EXPECT_EQ(a.current(), RunPhase::Stopped);

    RunState b; b.run(); b.reset();
    EXPECT_EQ(b.current(), RunPhase::Stopped);

    RunState c; c.run(); c.pause(); c.reset();
    EXPECT_EQ(c.current(), RunPhase::Stopped);
}

TEST(RunState, RunFromRunningIsIdempotentNotAnError) {
    // Double-clicking Run should not blow up — that's a fairly common
    // user mistake and there's nothing meaningful for it to do.
    RunState s;
    s.run();
    EXPECT_NO_THROW(s.run());
    EXPECT_EQ(s.current(), RunPhase::Running);
}

TEST(RunState, PauseFromStoppedThrows) {
    // Pausing a stopped sim is meaningless and signals a UI bug — the
    // Pause button should be disabled in this state. Throw so the test
    // / the assert layer in debug catches the path.
    RunState s;
    EXPECT_THROW(s.pause(), std::logic_error);
}

TEST(RunState, PauseFromPausedThrows) {
    RunState s;
    s.run();
    s.pause();
    EXPECT_THROW(s.pause(), std::logic_error);
}

TEST(RunState, ResetIsIdempotent) {
    RunState s;
    EXPECT_NO_THROW(s.reset());
    EXPECT_NO_THROW(s.reset());
    EXPECT_EQ(s.current(), RunPhase::Stopped);
}
