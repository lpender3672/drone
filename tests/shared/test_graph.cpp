#include <gtest/gtest.h>
#include <stdexcept>
#include <type_traits>
#include "core/graph.hpp"
#include "mocks/mock_block.h"

using shared::Graph;
using mocks::MockBlock;

TEST(Graph, NewGraphIsEmpty) {
    Graph g;
    EXPECT_EQ(g.block_count(), 0u);
    EXPECT_EQ(g.edge_count(), 0u);
    EXPECT_TRUE(g.edges().empty());
}

TEST(Graph, AddBlockReturnsNonNullPointer) {
    Graph g;
    auto* b = g.add_block(std::make_unique<MockBlock<>>("a"));
    EXPECT_NE(b, nullptr);
    EXPECT_EQ(b->name(), "a");
}

TEST(Graph, AddBlockIncrementsBlockCount) {
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));
    g.add_block(std::make_unique<MockBlock<>>("b"));
    EXPECT_EQ(g.block_count(), 2u);
}

TEST(Graph, FindReturnsBlockByName) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    EXPECT_EQ(g.find("a"), a);
    EXPECT_EQ(g.find("b"), b);
    EXPECT_EQ(g.find("missing"), nullptr);
}

TEST(Graph, AddBlockRejectsDuplicateName) {
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));
    EXPECT_THROW(g.add_block(std::make_unique<MockBlock<>>("a")),
                 std::invalid_argument);
}

TEST(Graph, ConnectRecordsSingleEdge) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));

    g.connect(*a, a->out(), *b, b->in());

    EXPECT_EQ(g.edge_count(), 1u);
    const auto& e = g.edges().front();
    EXPECT_EQ(e.src_block, a);
    EXPECT_EQ(e.dst_block, b);
    EXPECT_EQ(e.src_port, "out");
    EXPECT_EQ(e.dst_port, "in");
}

TEST(Graph, ConnectWiresUnderlyingPort) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));

    g.connect(*a, a->out(), *b, b->in());

    a->out().set(42.0);
    EXPECT_DOUBLE_EQ(b->in().get(), 42.0);  // zero-copy through the source pointer
}

TEST(Graph, ConnectMultipleEdgesAccumulate) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));
    auto* d = g.add_block(std::make_unique<MockBlock<>>("d"));

    g.connect(*a, a->out(), *b, b->in());
    g.connect(*a, a->out(), *c, c->in());
    g.connect(*a, a->out(), *d, d->in());

    EXPECT_EQ(g.edge_count(), 3u);
}

TEST(Graph, OutgoingReturnsEdgesFromSource) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));

    g.connect(*a, a->out(), *b, b->in());
    g.connect(*a, a->out(), *c, c->in());

    auto out = g.outgoing(a);
    EXPECT_EQ(out.size(), 2u);
    for (const auto& e : out) EXPECT_EQ(e.src_block, a);
}

TEST(Graph, IncomingReturnsEdgesToDest) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));

    // Two edges into c. Note: the second connect overwrites c.in's source
    // pointer at the port level — Graph's edge bookkeeping records both
    // regardless. (Tier 2 slice 2+ may reject duplicate inputs.)
    g.connect(*a, a->out(), *c, c->in());
    g.connect(*b, b->out(), *c, c->in());

    auto in = g.incoming(c);
    EXPECT_EQ(in.size(), 2u);
    for (const auto& e : in) EXPECT_EQ(e.dst_block, c);
}

TEST(Graph, IncomingEmptyForUnconnectedBlock) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    EXPECT_TRUE(g.incoming(b).empty());
    EXPECT_TRUE(g.outgoing(a).empty());
}

TEST(Graph, ConnectRejectsBlockNotInGraph) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    MockBlock<> stranger("stranger");

    EXPECT_THROW(g.connect(*a, a->out(), stranger, stranger.in()),
                 std::invalid_argument);
    EXPECT_THROW(g.connect(stranger, stranger.out(), *a, a->in()),
                 std::invalid_argument);
    EXPECT_EQ(g.edge_count(), 0u);  // failed connects don't leave half-edges
}

// Locks down the API invariant: Graph owns unique_ptrs and exposes raw
// pointers as stable handles into its tables. Copying would either alias
// the unique_ptrs (UB) or deep-clone the blocks (silently breaking those
// pointer handles). Either way, copying is wrong; this static_assert
// catches an accidental relaxation of that contract.
TEST(Graph, GraphIsNonCopyable) {
    static_assert(!std::is_copy_constructible_v<Graph>,
                  "Graph must remain non-copy-constructible");
    static_assert(!std::is_copy_assignable_v<Graph>,
                  "Graph must remain non-copy-assignable");
    SUCCEED();
}

TEST(Graph, IntegrationTwoBlockPipelineTicks) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    g.connect(*a, a->out(), *b, b->in());

    a->in().set(7.0);
    // Tick all blocks in declaration order via the blocks() accessor —
    // graph-driven scheduling proper arrives in slice 4 (topological sort).
    for (const auto& blk : g.blocks()) blk->update(0);

    EXPECT_DOUBLE_EQ(b->out().get(), 7.0);
    EXPECT_EQ(a->update_count, 1);
    EXPECT_EQ(b->update_count, 1);
}

// ============================================================================
// Slice 2: string-keyed Graph::connect (uses IPort lookup + virtual dispatch)
// ============================================================================

TEST(Graph, ConnectByStringWiresPortAndRecordsEdge) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));

    g.connect("a.out", "b.in");

    EXPECT_EQ(g.edge_count(), 1u);
    const auto& e = g.edges().front();
    EXPECT_EQ(e.src_block, a);
    EXPECT_EQ(e.dst_block, b);
    EXPECT_EQ(e.src_port, "out");
    EXPECT_EQ(e.dst_port, "in");

    // Zero-copy: writes to a.out flow to b.in.
    a->out().set(99.0);
    EXPECT_DOUBLE_EQ(b->in().get(), 99.0);
}

TEST(Graph, ConnectByStringRejectsMissingBlock) {
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));

    EXPECT_THROW(g.connect("ghost.out", "a.in"),  std::invalid_argument);
    EXPECT_THROW(g.connect("a.out",     "ghost.in"), std::invalid_argument);
    EXPECT_EQ(g.edge_count(), 0u);
}

TEST(Graph, ConnectByStringRejectsMissingPort) {
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));
    g.add_block(std::make_unique<MockBlock<>>("b"));

    EXPECT_THROW(g.connect("a.fake_out", "b.in"),      std::invalid_argument);
    EXPECT_THROW(g.connect("a.out",      "b.fake_in"), std::invalid_argument);
    EXPECT_EQ(g.edge_count(), 0u);
}

TEST(Graph, ConnectByStringRejectsTypeMismatch) {
    Graph g;
    g.add_block(std::make_unique<MockBlock<int>>("a"));
    g.add_block(std::make_unique<MockBlock<double>>("b"));

    EXPECT_THROW(g.connect("a.out", "b.in"), std::invalid_argument);
    EXPECT_EQ(g.edge_count(), 0u);
}

TEST(Graph, ConnectByStringRejectsDirectionMismatch) {
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));
    g.add_block(std::make_unique<MockBlock<>>("b"));

    // input as source: a.in is an input port, not a valid source.
    EXPECT_THROW(g.connect("a.in", "b.in"), std::invalid_argument);
    // output as destination: b.out is an output, not a valid sink.
    EXPECT_THROW(g.connect("a.out", "b.out"), std::invalid_argument);
    EXPECT_EQ(g.edge_count(), 0u);
}

TEST(Graph, ConnectByStringRejectsMalformedPath) {
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));
    g.add_block(std::make_unique<MockBlock<>>("b"));

    // No '.' separator — can't tell what's a block name and what's a port.
    EXPECT_THROW(g.connect("aout", "b.in"), std::invalid_argument);
    EXPECT_THROW(g.connect("a.out", "bin"), std::invalid_argument);
    EXPECT_EQ(g.edge_count(), 0u);
}

TEST(Graph, IntegrationByStringPipelineTicks) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));

    g.connect("a.out", "b.in");

    a->in().set(13.5);
    for (const auto& blk : g.blocks()) blk->update(0);

    EXPECT_DOUBLE_EQ(b->out().get(), 13.5);
}
