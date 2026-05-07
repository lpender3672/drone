#include <gtest/gtest.h>

#include <frame_clock.hpp>

using editor::FrameClock;

// FrameClock is a pure, time-source-agnostic counter for the editor's
// render loop. Tests inject a monotonic time supplier so they don't
// depend on wall clock — same pattern the block runtime uses.

TEST(FrameClock, DefaultConstructedHasZeroFramesAndZeroDt) {
    FrameClock c;
    EXPECT_EQ(c.frame_count(), 0u);
    EXPECT_DOUBLE_EQ(c.dt_ms(), 0.0);
}

TEST(FrameClock, FirstTickAdvancesCountButLeavesDtZero) {
    // No prior tick → no interval to measure. Same convention as
    // shared::Block (first update returns update_period for dt to avoid
    // divide-by-zero or jumpy d-terms downstream).
    FrameClock c;
    c.tick(/*now_us=*/1'000'000);
    EXPECT_EQ(c.frame_count(), 1u);
    EXPECT_DOUBLE_EQ(c.dt_ms(), 0.0);
}

TEST(FrameClock, SecondTickReportsIntervalSinceFirst) {
    FrameClock c;
    c.tick(1'000'000);   // t = 1.000 s
    c.tick(1'016'667);   // t = 1.0167 s → 16.667 ms
    EXPECT_EQ(c.frame_count(), 2u);
    EXPECT_NEAR(c.dt_ms(), 16.667, 1e-3);
}

TEST(FrameClock, DtTracksMostRecentInterval) {
    // dt_ms reports the *latest* frame interval, not a running average.
    // Lets the overlay show real-time frame pacing.
    FrameClock c;
    c.tick(0);
    c.tick(20'000);   //  20 ms
    c.tick(25'000);   //   5 ms
    c.tick(50'000);   //  25 ms
    EXPECT_EQ(c.frame_count(), 4u);
    EXPECT_NEAR(c.dt_ms(), 25.0, 1e-9);
}

TEST(FrameClock, NonMonotonicTickThrows) {
    // GLFW's monotonic clock is, well, monotonic — but defensive: if the
    // caller ever feeds a backwards timestamp it's a bug, not a UI glitch.
    FrameClock c;
    c.tick(1000);
    EXPECT_THROW(c.tick(500), std::invalid_argument);
}

TEST(FrameClock, ResetReturnsToInitialState) {
    FrameClock c;
    c.tick(1000);
    c.tick(2000);
    c.reset();
    EXPECT_EQ(c.frame_count(), 0u);
    EXPECT_DOUBLE_EQ(c.dt_ms(), 0.0);

    // After reset, the next tick is a "first" tick again — dt stays 0.
    c.tick(99'999);
    EXPECT_EQ(c.frame_count(), 1u);
    EXPECT_DOUBLE_EQ(c.dt_ms(), 0.0);
}

TEST(FrameClock, FpsConveniencInverseOfDt) {
    // 16.667 ms → 60 fps. fps() returns 0 on the first tick (no interval).
    FrameClock c;
    c.tick(0);
    EXPECT_DOUBLE_EQ(c.fps(), 0.0);
    c.tick(16'667);
    EXPECT_NEAR(c.fps(), 60.0, 0.01);
}
