#include <gtest/gtest.h>
#include <stdexcept>
#include "core/block_factory.hpp"
#include "core/graph.hpp"
#include "mocks/mock_block.h"

using shared::BlockFactory;
using shared::BlockParams;
using shared::Graph;
using mocks::MockBlock;

namespace {

// Trivial factory for MockBlock<double>. Reads "seed_value" out of params
// (default 0.0) and stamps it onto the block's input port so tests can
// see params actually flowed through.
auto mock_double_ctor() {
    return [](const std::string& name, const BlockParams& params) -> std::unique_ptr<shared::Block> {
        auto b = std::make_unique<MockBlock<double>>(name);
        if (params.has("seed_value")) {
            b->in().set(params.get_double("seed_value"));
        }
        return b;
    };
}

} // namespace

TEST(BlockFactory, NewFactoryHasNoTypes) {
    BlockFactory f;
    EXPECT_EQ(f.type_count(), 0u);
    EXPECT_FALSE(f.has_type("anything"));
}

TEST(BlockFactory, RegisterAddsType) {
    BlockFactory f;
    f.register_type("mock", mock_double_ctor());
    EXPECT_EQ(f.type_count(), 1u);
    EXPECT_TRUE(f.has_type("mock"));
}

TEST(BlockFactory, CreateStampsTypeNameOnBlock) {
    // The factory key is the round-trip identity that dump_graph_json
    // emits, so each created block must carry it. Direct C++
    // construction (skipping the factory) leaves type_name empty.
    BlockFactory f;
    f.register_type("mock", mock_double_ctor());

    auto block = f.create("mock", "alpha");
    EXPECT_EQ(block->type_name(), "mock");
    EXPECT_EQ(block->name(),      "alpha");

    MockBlock<double> direct("direct");
    EXPECT_EQ(direct.type_name(), "");
}

TEST(BlockFactory, CreateProducesBlockWithGivenName) {
    BlockFactory f;
    f.register_type("mock", mock_double_ctor());

    auto b = f.create("mock", "alpha");

    ASSERT_NE(b, nullptr);
    EXPECT_EQ(b->name(), "alpha");
}

TEST(BlockFactory, CreateUnknownTypeThrows) {
    BlockFactory f;
    EXPECT_THROW(f.create("nope", "x"), std::invalid_argument);
}

TEST(BlockFactory, RegisterDuplicateTypeThrows) {
    BlockFactory f;
    f.register_type("mock", mock_double_ctor());
    EXPECT_THROW(f.register_type("mock", mock_double_ctor()), std::invalid_argument);
}

TEST(BlockFactory, CreateForwardsParamsToCtor) {
    BlockFactory f;
    f.register_type("mock", mock_double_ctor());

    BlockParams p;
    p.set("seed_value", 42.0);

    auto b = f.create("mock", "a", p);
    auto* mock = dynamic_cast<MockBlock<double>*>(b.get());
    ASSERT_NE(mock, nullptr);
    EXPECT_DOUBLE_EQ(mock->in().get(), 42.0);
}

TEST(BlockFactory, CreateProducesIndependentInstances) {
    BlockFactory f;
    f.register_type("mock", mock_double_ctor());

    auto a = f.create("mock", "a");
    auto b = f.create("mock", "b");

    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_NE(a.get(), b.get());
    EXPECT_NE(a->name(), b->name());
}

TEST(BlockParams, GetDoubleDefaultsWhenAbsent) {
    BlockParams p;
    EXPECT_FALSE(p.has("missing"));
    EXPECT_DOUBLE_EQ(p.get_double("missing"),       0.0);
    EXPECT_DOUBLE_EQ(p.get_double("missing", 7.5),  7.5);
}

TEST(BlockParams, SetAndGetRoundTrips) {
    BlockParams p;
    p.set("kp", 0.25);
    p.set("ki", 0.01);
    EXPECT_TRUE(p.has("kp"));
    EXPECT_DOUBLE_EQ(p.get_double("kp"), 0.25);
    EXPECT_DOUBLE_EQ(p.get_double("ki"), 0.01);
}

// Integration: the factory's output is a real Block usable in a Graph.
TEST(BlockFactory, CreateAndAddToGraphAndConnectByStringTicks) {
    BlockFactory f;
    f.register_type("mock", mock_double_ctor());

    Graph g;
    BlockParams p_a;
    p_a.set("seed_value", 11.0);

    // Factory returns unique_ptr<Block>; Graph::add_block deduces T=Block.
    // Downcast to the concrete mock for typed assertions on the test side.
    auto* a = dynamic_cast<MockBlock<double>*>(
                  g.add_block(f.create("mock", "a", p_a)));
    auto* b = dynamic_cast<MockBlock<double>*>(
                  g.add_block(f.create("mock", "b")));
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);

    g.connect("a.out", "b.in");

    for (const auto& blk : g.blocks()) blk->update(0);

    EXPECT_DOUBLE_EQ(b->out().get(), 11.0);
    EXPECT_EQ(a->update_count, 1);
    EXPECT_EQ(b->update_count, 1);
}
