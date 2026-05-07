#include <gtest/gtest.h>

#include <recent_files.hpp>

using editor::RecentFiles;

// RecentFiles is a bounded most-recent-first list of file paths,
// deduplicated. Powers the editor's File > Recent submenu. Pure logic;
// the menu rendering layer in main.cpp consumes entries() each frame.

TEST(RecentFiles, DefaultIsEmptyWithCapEight) {
    RecentFiles r;
    EXPECT_EQ(r.size(), 0u);
    EXPECT_TRUE(r.entries().empty());
    EXPECT_EQ(r.max_size(), 8u);
}

TEST(RecentFiles, ConstructWithExplicitCap) {
    RecentFiles r(/*max=*/3);
    EXPECT_EQ(r.max_size(), 3u);
}

TEST(RecentFiles, PushAddsAndOrdersMostRecentFirst) {
    RecentFiles r;
    r.push("a.json");
    r.push("b.json");
    r.push("c.json");
    ASSERT_EQ(r.size(), 3u);
    const auto& e = r.entries();
    EXPECT_EQ(e[0], "c.json");
    EXPECT_EQ(e[1], "b.json");
    EXPECT_EQ(e[2], "a.json");
}

TEST(RecentFiles, RepeatedPushMovesPathToFrontWithoutDuplicating) {
    RecentFiles r;
    r.push("a.json");
    r.push("b.json");
    r.push("a.json");  // existing → move to front
    ASSERT_EQ(r.size(), 2u);
    EXPECT_EQ(r.entries()[0], "a.json");
    EXPECT_EQ(r.entries()[1], "b.json");
}

TEST(RecentFiles, EvictsOldestPastCap) {
    RecentFiles r(/*max=*/3);
    r.push("a"); r.push("b"); r.push("c"); r.push("d");
    ASSERT_EQ(r.size(), 3u);
    EXPECT_EQ(r.entries()[0], "d");
    EXPECT_EQ(r.entries()[1], "c");
    EXPECT_EQ(r.entries()[2], "b");
    // "a" got evicted.
}

TEST(RecentFiles, DedupTakesPriorityOverCap) {
    // Re-pushing an existing path moves it to the front and shouldn't
    // count as a new entry — the list mustn't grow past cap as a side
    // effect of re-opening the same file repeatedly.
    RecentFiles r(/*max=*/3);
    r.push("a"); r.push("b"); r.push("c");
    r.push("a"); r.push("b"); r.push("c");
    EXPECT_EQ(r.size(), 3u);
}

TEST(RecentFiles, ClearEmptiesTheList) {
    RecentFiles r;
    r.push("a"); r.push("b");
    r.clear();
    EXPECT_EQ(r.size(), 0u);
    EXPECT_TRUE(r.entries().empty());
}

TEST(RecentFiles, EmptyPathIsRejected) {
    // Empty path is a programming bug somewhere upstream — better to
    // reject loudly than to silently end up with a "(empty)" recent entry.
    RecentFiles r;
    EXPECT_THROW(r.push(""), std::invalid_argument);
    EXPECT_EQ(r.size(), 0u);
}

TEST(RecentFiles, ZeroCapIsRejected) {
    EXPECT_THROW(RecentFiles(0), std::invalid_argument);
}
