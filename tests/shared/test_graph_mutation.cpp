#include <gtest/gtest.h>

#include <core/graph.hpp>

#include "mocks/mock_block.h"

using mocks::MockBlock;
using shared::Graph;

// Mutation-after-build covers `disconnect` and `remove_block`. Both
// land in slice 4 as the runtime prerequisites for live editing in
// the editor (slice 5).

namespace {

struct TwoWired {
    Graph g;
    MockBlock<>* a;
    MockBlock<>* b;

    TwoWired() {
        a = g.add_block(std::make_unique<MockBlock<>>("a"));
        b = g.add_block(std::make_unique<MockBlock<>>("b"));
        g.connect(*a, a->out(), *b, b->in());
    }
};

} // namespace

// ---- disconnect -----------------------------------------------------------

TEST(GraphDisconnect, RemovesEdgeAndNullsDownstreamSource) {
    TwoWired t;

    // Sanity — before disconnect, the input reads through the source ptr.
    t.a->out().set(7.0);
    EXPECT_DOUBLE_EQ(t.b->in().get(), 7.0);
    EXPECT_TRUE(t.b->in().connected);

    t.g.disconnect("a.out", "b.in");

    EXPECT_EQ(t.g.edge_count(), 0u);
    EXPECT_FALSE(t.b->in().connected);

    // Source pointer must be nulled — get() now falls back to the
    // input's own `value_` (default-constructed double = 0). If the
    // pointer were still wired, get() would see a's new value (99).
    t.a->out().set(99.0);
    EXPECT_DOUBLE_EQ(t.b->in().get(), 0.0);
}

TEST(GraphDisconnect, BumpsVersion) {
    TwoWired t;
    const std::size_t before = t.g.version();
    t.g.disconnect("a.out", "b.in");
    EXPECT_GT(t.g.version(), before);
}

TEST(GraphDisconnect, ClearsIncomingAndOutgoingIndexEntries) {
    TwoWired t;
    ASSERT_EQ(t.g.outgoing(t.a).size(), 1u);
    ASSERT_EQ(t.g.incoming(t.b).size(), 1u);

    t.g.disconnect("a.out", "b.in");

    EXPECT_TRUE(t.g.outgoing(t.a).empty());
    EXPECT_TRUE(t.g.incoming(t.b).empty());
}

TEST(GraphDisconnect, NonExistentEdgeThrows) {
    TwoWired t;
    EXPECT_THROW(t.g.disconnect("a.in", "b.out"),  std::invalid_argument);
    EXPECT_THROW(t.g.disconnect("a.out", "b.out"), std::invalid_argument);
    EXPECT_THROW(t.g.disconnect("missing.out", "b.in"), std::invalid_argument);
    EXPECT_EQ(t.g.edge_count(), 1u);  // graph untouched on failure
}

TEST(GraphDisconnect, MalformedPathThrows) {
    TwoWired t;
    EXPECT_THROW(t.g.disconnect("a_out", "b.in"), std::invalid_argument);
    EXPECT_THROW(t.g.disconnect("a.out", "b_in"), std::invalid_argument);
}

TEST(GraphDisconnect, OneOfManyParallelEdgesGoes) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));
    g.connect(*a, a->out(), *b, b->in());
    g.connect(*a, a->out(), *c, c->in());

    ASSERT_EQ(g.edge_count(), 2u);
    g.disconnect("a.out", "b.in");

    EXPECT_EQ(g.edge_count(), 1u);
    EXPECT_FALSE(b->in().connected);
    EXPECT_TRUE (c->in().connected);  // sibling edge survives
    EXPECT_EQ(g.outgoing(a).size(), 1u);
}

// ---- remove_block ---------------------------------------------------------

TEST(GraphRemoveBlock, DropsBlockAndAllEdges) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));
    g.connect(*a, a->out(), *b, b->in());
    g.connect(*b, b->out(), *c, c->in());

    g.remove_block(b);  // both edges referenced `b` — both must go.

    EXPECT_EQ(g.block_count(), 2u);
    EXPECT_EQ(g.edge_count(),  0u);
    EXPECT_EQ(g.find("b"),     nullptr);
    // Any survivor's input ports that pointed at the removed block
    // must have their source pointer nulled — otherwise next read
    // dereferences a freed OutputPort.
    EXPECT_FALSE(c->in().connected);
}

TEST(GraphRemoveBlock, BumpsVersion) {
    TwoWired t;
    const std::size_t before = t.g.version();
    t.g.remove_block(t.a);
    EXPECT_GT(t.g.version(), before);
}

TEST(GraphRemoveBlock, RemovingSinkLeavesUpstreamUntouched) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    g.connect(*a, a->out(), *b, b->in());

    g.remove_block(b);

    EXPECT_EQ(g.block_count(), 1u);
    EXPECT_EQ(g.edge_count(),  0u);
    // a is fine; outputs unaffected.
    EXPECT_EQ(g.find("a"), static_cast<shared::Block*>(a));
}

TEST(GraphRemoveBlock, StrangerThrows) {
    TwoWired t;
    MockBlock<> stranger("x");
    EXPECT_THROW(t.g.remove_block(&stranger), std::invalid_argument);
    EXPECT_EQ(t.g.block_count(), 2u);   // graph untouched
    EXPECT_EQ(t.g.edge_count(),  1u);
}

TEST(GraphRemoveBlock, IsolatedBlockComesOutCleanly) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    g.remove_block(a);
    EXPECT_EQ(g.block_count(), 0u);
    EXPECT_EQ(g.find("a"),     nullptr);
}

TEST(GraphRemoveBlock, AllowsAddingNewBlockWithSameNameAfterward) {
    // The name was the canonical handle — once the block is gone, it's
    // free to be reused. Without this, a long editing session would
    // accumulate forbidden names.
    Graph g;
    auto* a1 = g.add_block(std::make_unique<MockBlock<>>("a"));
    g.remove_block(a1);
    EXPECT_NO_THROW(g.add_block(std::make_unique<MockBlock<>>("a")));
}
