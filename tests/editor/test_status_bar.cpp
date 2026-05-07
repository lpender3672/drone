#include <gtest/gtest.h>

#include <status_bar.hpp>

using editor::StatusBar;
using editor::StatusLevel;

// StatusBar holds the latest user-visible message with a posted-time
// stamp. The render layer reads `current(now)` each frame; messages
// fade out after a configurable TTL so a transient error doesn't sit
// on screen forever. Pure logic; the bottom-pinned ImGui window in
// main.cpp consumes it.

TEST(StatusBar, FreshHasNoMessage) {
    StatusBar s;
    const auto m = s.current(/*now_us=*/0);
    EXPECT_FALSE(m.has_value());
}

TEST(StatusBar, PostedMessageIsCurrentImmediately) {
    StatusBar s;
    s.post(StatusLevel::Info, "hello", /*now_us=*/1'000'000);
    const auto m = s.current(/*now_us=*/1'000'000);
    ASSERT_TRUE(m.has_value());
    EXPECT_EQ(m->level,   StatusLevel::Info);
    EXPECT_EQ(m->message, "hello");
}

TEST(StatusBar, MessageFadesAfterTtl) {
    StatusBar s(/*ttl_us=*/5'000'000);  // 5 s
    s.post(StatusLevel::Info, "hi", /*now_us=*/0);

    EXPECT_TRUE (s.current(4'999'999).has_value());
    EXPECT_FALSE(s.current(5'000'001).has_value());
}

TEST(StatusBar, NewerMessageReplacesOlder) {
    StatusBar s;
    s.post(StatusLevel::Info,  "first",  /*now_us=*/0);
    s.post(StatusLevel::Error, "second", /*now_us=*/1'000'000);
    const auto m = s.current(1'000'000);
    ASSERT_TRUE(m.has_value());
    EXPECT_EQ(m->message, "second");
    EXPECT_EQ(m->level,   StatusLevel::Error);
}

TEST(StatusBar, ClearRemovesCurrentMessage) {
    StatusBar s;
    s.post(StatusLevel::Info, "x", 0);
    s.clear();
    EXPECT_FALSE(s.current(0).has_value());
}

TEST(StatusBar, DefaultTtlIsFiveSeconds) {
    // Spec test — locks the default in so a future tweak shows up here.
    StatusBar s;
    EXPECT_EQ(s.ttl_us(), 5'000'000u);
}

TEST(StatusBar, ZeroTtlIsRejected) {
    EXPECT_THROW(StatusBar(0), std::invalid_argument);
}

TEST(StatusBar, ErrorLevelDoesNotChangeFadeBehaviour) {
    // Slice 3 keeps the same TTL across levels — Error vs Info is just
    // a colour hint to the renderer. If we later want errors to stick
    // until acknowledged, this test would flip and document the change.
    StatusBar s(/*ttl_us=*/100'000);
    s.post(StatusLevel::Error, "boom", 0);
    EXPECT_FALSE(s.current(200'000).has_value());
}
