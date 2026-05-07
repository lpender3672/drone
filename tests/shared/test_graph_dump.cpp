#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <core/block_factory.hpp>
#include <core/graph.hpp>
#include <core/graph_loader.hpp>

#include "mocks/mock_block.h"

using shared::BlockFactory;
using shared::BlockParams;
using shared::Graph;
using mocks::MockBlock;

namespace {

shared::BlockFactory make_mock_factory() {
    BlockFactory f;
    f.register_type("mock",
        [](const std::string& name, const BlockParams&) {
            return std::make_unique<MockBlock<>>(name);
        });
    return f;
}

} // namespace

// dump_graph_json is the inverse of load_graph_json. Slice 4 contract:
// emit topology only — type_name, instance name, edges. Block params
// are NOT round-tripped (the factory's compile-time defaults apply on
// reload). A follow-up slice can extend BlockParams to round-trip.

TEST(DumpGraphJson, EmptyGraphProducesEmptyArrays) {
    Graph g;
    const auto j = nlohmann::json::parse(shared::dump_graph_json(g));
    ASSERT_TRUE(j.is_object());
    ASSERT_TRUE(j.contains("blocks"));
    ASSERT_TRUE(j.contains("edges"));
    EXPECT_TRUE(j["blocks"].is_array());
    EXPECT_TRUE(j["edges"].is_array());
    EXPECT_EQ(j["blocks"].size(), 0u);
    EXPECT_EQ(j["edges"].size(),  0u);
}

TEST(DumpGraphJson, BlocksWithoutEdges) {
    BlockFactory f = make_mock_factory();
    Graph g;
    g.add_block(f.create("mock", "alpha"));
    g.add_block(f.create("mock", "beta"));

    const auto j = nlohmann::json::parse(shared::dump_graph_json(g));
    ASSERT_EQ(j["blocks"].size(), 2u);

    EXPECT_EQ(j["blocks"][0]["type"], "mock");
    EXPECT_EQ(j["blocks"][0]["name"], "alpha");
    EXPECT_EQ(j["blocks"][1]["type"], "mock");
    EXPECT_EQ(j["blocks"][1]["name"], "beta");

    EXPECT_EQ(j["edges"].size(), 0u);
}

TEST(DumpGraphJson, EdgesEmittedAsBlockDotPort) {
    BlockFactory f = make_mock_factory();
    Graph g;
    auto* a = g.add_block(f.create("mock", "a"));
    auto* b = g.add_block(f.create("mock", "b"));
    auto* a_typed = dynamic_cast<MockBlock<>*>(a);
    auto* b_typed = dynamic_cast<MockBlock<>*>(b);
    ASSERT_TRUE(a_typed && b_typed);
    g.connect(*a, a_typed->out(), *b, b_typed->in());

    const auto j = nlohmann::json::parse(shared::dump_graph_json(g));
    ASSERT_EQ(j["edges"].size(), 1u);
    EXPECT_EQ(j["edges"][0]["from"], "a.out");
    EXPECT_EQ(j["edges"][0]["to"],   "b.in");
}

TEST(DumpGraphJson, BlockOrderMatchesInsertionOrder) {
    BlockFactory f = make_mock_factory();
    Graph g;
    g.add_block(f.create("mock", "z"));
    g.add_block(f.create("mock", "a"));
    g.add_block(f.create("mock", "m"));
    const auto j = nlohmann::json::parse(shared::dump_graph_json(g));
    EXPECT_EQ(j["blocks"][0]["name"], "z");
    EXPECT_EQ(j["blocks"][1]["name"], "a");
    EXPECT_EQ(j["blocks"][2]["name"], "m");
}

TEST(DumpGraphJson, BlockWithoutTypeNameThrows) {
    // A block constructed directly (not via factory) has no type name.
    // Emitting it would produce JSON the loader can't round-trip, so
    // refuse explicitly.
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("orphan"));
    EXPECT_THROW(shared::dump_graph_json(g), std::invalid_argument);
}

TEST(DumpGraphJson, OutputIsValidJson) {
    BlockFactory f = make_mock_factory();
    Graph g;
    g.add_block(f.create("mock", "a"));
    g.add_block(f.create("mock", "b"));
    const std::string text = shared::dump_graph_json(g);
    // Should parse without throwing — implicit assertion in parse.
    EXPECT_NO_THROW(nlohmann::json::parse(text));
}
