#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include <app_state.hpp>
#include <core/block_factory.hpp>

#include "../mocks/mock_block.h"

using editor::AppState;
using editor::StatusLevel;
using mocks::MockBlock;

namespace {

// Per-test temp file. Picks the GTest test name as a unique suffix so
// parallel runs don't trample each other.
std::string temp_path(const std::string& suffix) {
    const auto* info =
        ::testing::UnitTest::GetInstance()->current_test_info();
    return std::string("/tmp/drone_test_") + info->name() + "_" + suffix + ".json";
}

void write_file(const std::string& path, const std::string& text) {
    std::ofstream out(path);
    out << text;
}

void remove_file(const std::string& path) {
    std::remove(path.c_str());
}

// Minimal factory for tests — registers just `mock` so JSON specs don't
// need to invoke the full sim block universe. Keeps the AppState unit
// narrow: we test the load/error/reload path mechanics, not the
// quadrotor registrations (those have their own integration test).
shared::BlockFactory make_mock_factory() {
    shared::BlockFactory f;
    f.register_type("mock",
        [](const std::string& name, const shared::BlockParams&) {
            return std::make_unique<MockBlock<>>(name);
        });
    return f;
}

constexpr const char* kTwoBlockGraph = R"({
    "blocks": [
        { "type": "mock", "name": "a" },
        { "type": "mock", "name": "b" }
    ],
    "edges": [
        { "from": "a.out", "to": "b.in" }
    ]
})";

} // namespace

TEST(AppState, FreshHasEmptyGraphAndNoCurrentPath) {
    AppState app(make_mock_factory());
    EXPECT_EQ(app.graph().block_count(), 0u);
    EXPECT_TRUE(app.current_path().empty());
    EXPECT_FALSE(app.status_bar().current(0).has_value());
}

TEST(AppState, LoadValidPathPopulatesGraphAndPostsInfo) {
    AppState app(make_mock_factory());
    const std::string path = temp_path("valid");
    write_file(path, kTwoBlockGraph);

    EXPECT_TRUE(app.load_graph(path, /*now_us=*/1'000'000));
    EXPECT_EQ(app.graph().block_count(), 2u);
    EXPECT_EQ(app.graph().edge_count(),  1u);
    EXPECT_EQ(app.current_path(), path);

    const auto msg = app.status_bar().current(1'000'000);
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->level, StatusLevel::Info);

    remove_file(path);
}

TEST(AppState, LoadValidPathPushesToRecentFiles) {
    AppState app(make_mock_factory());
    const std::string path = temp_path("recent");
    write_file(path, kTwoBlockGraph);

    EXPECT_TRUE(app.load_graph(path, 0));
    ASSERT_EQ(app.recent_files().size(), 1u);
    EXPECT_EQ(app.recent_files().entries()[0], path);

    remove_file(path);
}

TEST(AppState, LoadMissingPathFailsLeavesPriorGraph) {
    AppState app(make_mock_factory());
    const std::string ok_path = temp_path("kept");
    write_file(ok_path, kTwoBlockGraph);
    ASSERT_TRUE(app.load_graph(ok_path, 0));

    // Now try to load a path that doesn't exist. The prior graph and
    // path must survive — partial state is the worst UX.
    EXPECT_FALSE(app.load_graph("/no/such/file.json", 1'000'000));
    EXPECT_EQ(app.graph().block_count(), 2u);
    EXPECT_EQ(app.current_path(), ok_path);

    const auto msg = app.status_bar().current(1'000'000);
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->level, StatusLevel::Error);

    // Recent-files isn't touched on failure either.
    EXPECT_EQ(app.recent_files().size(), 1u);
    EXPECT_EQ(app.recent_files().entries()[0], ok_path);

    remove_file(ok_path);
}

TEST(AppState, LoadMalformedJsonFailsAtomically) {
    AppState app(make_mock_factory());
    const std::string path = temp_path("bad");
    write_file(path, "{ this is not json }");

    EXPECT_FALSE(app.load_graph(path, 0));
    EXPECT_EQ(app.graph().block_count(), 0u);
    EXPECT_TRUE(app.current_path().empty());

    const auto msg = app.status_bar().current(0);
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->level, StatusLevel::Error);

    remove_file(path);
}

TEST(AppState, ReloadRereadsCurrentPath) {
    AppState app(make_mock_factory());
    const std::string path = temp_path("reload");
    write_file(path, kTwoBlockGraph);
    ASSERT_TRUE(app.load_graph(path, 0));

    // Mutate the file on disk: drop one block. Reload should pick up
    // the new contents — proves it's actually re-reading and not just
    // returning the cached graph.
    constexpr const char* kSingleBlock = R"({
        "blocks": [{ "type": "mock", "name": "a" }]
    })";
    write_file(path, kSingleBlock);

    EXPECT_TRUE(app.reload(1'000'000));
    EXPECT_EQ(app.graph().block_count(), 1u);
    EXPECT_EQ(app.graph().edge_count(),  0u);

    remove_file(path);
}

TEST(AppState, ReloadIsNoOpWithNoCurrentPath) {
    AppState app(make_mock_factory());
    // No prior load_graph.
    EXPECT_FALSE(app.reload(0));
    EXPECT_EQ(app.graph().block_count(), 0u);
}

TEST(AppState, ClearEmptiesGraphAndCurrentPath) {
    AppState app(make_mock_factory());
    const std::string path = temp_path("clear");
    write_file(path, kTwoBlockGraph);
    ASSERT_TRUE(app.load_graph(path, 0));

    app.clear();
    EXPECT_EQ(app.graph().block_count(), 0u);
    EXPECT_TRUE(app.current_path().empty());
    // Clearing the open document doesn't wipe the recent-files list —
    // the user may want to reopen something they just closed.
    EXPECT_EQ(app.recent_files().size(), 1u);

    remove_file(path);
}

TEST(AppState, SaveGraphWritesValidLoadableJson) {
    AppState app(make_mock_factory());
    const std::string in_path  = temp_path("save_in");
    const std::string out_path = temp_path("save_out");
    write_file(in_path, kTwoBlockGraph);
    ASSERT_TRUE(app.load_graph(in_path, 0));

    EXPECT_TRUE(app.save_graph(out_path, 1'000'000));
    EXPECT_EQ(app.current_path(), out_path);  // save updates current_path

    // The output file must be loadable by a fresh AppState.
    AppState reloaded(make_mock_factory());
    EXPECT_TRUE(reloaded.load_graph(out_path, 0));
    EXPECT_EQ(reloaded.graph().block_count(), 2u);
    EXPECT_EQ(reloaded.graph().edge_count(),  1u);

    remove_file(in_path);
    remove_file(out_path);
}

TEST(AppState, SaveGraphFailsCleanlyOnUnwritablePath) {
    AppState app(make_mock_factory());
    const std::string ok_path = temp_path("save_keep");
    write_file(ok_path, kTwoBlockGraph);
    ASSERT_TRUE(app.load_graph(ok_path, 0));

    EXPECT_FALSE(app.save_graph("/no/such/dir/out.json", 1'000'000));
    EXPECT_EQ(app.current_path(), ok_path);  // unchanged on failure

    const auto msg = app.status_bar().current(1'000'000);
    ASSERT_TRUE(msg.has_value());
    EXPECT_EQ(msg->level, StatusLevel::Error);

    remove_file(ok_path);
}

TEST(AppState, RepeatLoadOfSamePathStaysSingleRecentEntry) {
    AppState app(make_mock_factory());
    const std::string path = temp_path("repeat");
    write_file(path, kTwoBlockGraph);
    ASSERT_TRUE(app.load_graph(path, 0));
    ASSERT_TRUE(app.load_graph(path, 1'000'000));
    EXPECT_EQ(app.recent_files().size(), 1u);

    remove_file(path);
}
