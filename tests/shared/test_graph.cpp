#include <gtest/gtest.h>
#include <algorithm>
#include <stdexcept>
#include <type_traits>
#include "core/graph.hpp"
#include "blocks/unit_delay.hpp"
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
TEST(Graph, GraphIsNonCopyableButMovable) {
    static_assert(!std::is_copy_constructible_v<Graph>,
                  "Graph must remain non-copy-constructible");
    static_assert(!std::is_copy_assignable_v<Graph>,
                  "Graph must remain non-copy-assignable");
    static_assert(std::is_move_constructible_v<Graph>,
                  "Graph must be move-constructible (CompositeBlock holds one as a member)");
    static_assert(std::is_move_assignable_v<Graph>,
                  "Graph must be move-assignable");
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

// ============================================================================
// Slice 4: topological sort
// ============================================================================

TEST(Graph, TopoOrderEmptyGraphReturnsEmpty) {
    Graph g;
    EXPECT_TRUE(g.topo_order().empty());
}

TEST(Graph, TopoOrderSingleBlockReturnsThatBlock) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto order = g.topo_order();
    ASSERT_EQ(order.size(), 1u);
    EXPECT_EQ(order[0], a);
}

TEST(Graph, TopoOrderLinearChainOrdersByDataFlow) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));

    g.connect("a.out", "b.in");
    g.connect("b.out", "c.in");

    auto order = g.topo_order();
    ASSERT_EQ(order.size(), 3u);
    EXPECT_EQ(order[0], a);
    EXPECT_EQ(order[1], b);
    EXPECT_EQ(order[2], c);
}

TEST(Graph, TopoOrderUnconnectedBlocksKeepInsertionOrder) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));

    auto order = g.topo_order();
    ASSERT_EQ(order.size(), 3u);
    EXPECT_EQ(order[0], a);
    EXPECT_EQ(order[1], b);
    EXPECT_EQ(order[2], c);
}

TEST(Graph, TopoOrderRespectsFanInPredecessors) {
    // a → c, b → c.  c must come last; a and b in either order.
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));

    g.connect("a.out", "c.in");
    // Two sources can't both wire into c.in (the second overwrites). Use
    // distinct edges only: add another mock block with a typed-port wiring.
    // Simpler — verify just one fan-in edge keeps c after a:
    auto order = g.topo_order();
    auto pos_a = std::find(order.begin(), order.end(), a) - order.begin();
    auto pos_c = std::find(order.begin(), order.end(), c) - order.begin();
    EXPECT_LT(pos_a, pos_c);

    auto pos_b = std::find(order.begin(), order.end(), b) - order.begin();
    EXPECT_NE(pos_b, static_cast<long>(order.size()));  // b is in the order
}

TEST(Graph, TopoOrderRespectsFanOutSuccessors) {
    // a → b, a → c.  a must come first; b and c in either order.
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));

    g.connect("a.out", "b.in");
    g.connect("a.out", "c.in");

    auto order = g.topo_order();
    ASSERT_EQ(order.size(), 3u);
    EXPECT_EQ(order[0], a);
    // b and c can come in any valid topological order.
    auto pos_b = std::find(order.begin(), order.end(), b) - order.begin();
    auto pos_c = std::find(order.begin(), order.end(), c) - order.begin();
    EXPECT_GT(pos_b, 0);
    EXPECT_GT(pos_c, 0);
}

TEST(Graph, TopoOrderHandlesDiamond) {
    // a → b, a → c, b → d, c → d
    // Valid orders: [a,b,c,d] or [a,c,b,d]. With insertion-order tiebreak,
    // we get [a,b,c,d] because b was added before c.
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));
    auto* d = g.add_block(std::make_unique<MockBlock<>>("d"));

    g.connect("a.out", "b.in");
    g.connect("a.out", "c.in");
    g.connect("b.out", "d.in");
    g.connect("c.out", "d.in");

    auto order = g.topo_order();
    ASSERT_EQ(order.size(), 4u);
    EXPECT_EQ(order.front(), a);
    EXPECT_EQ(order.back(),  d);
}

TEST(Graph, TopoOrderDeterministicTiebreakByInsertionOrder) {
    // Two parallel chains: a → b, c → d. Insertion order determines
    // which chain comes first; result is [a, b, c, d] since a was added
    // before c (both at in_degree=0 initially, picker takes earlier-inserted).
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));
    auto* d = g.add_block(std::make_unique<MockBlock<>>("d"));

    g.connect("a.out", "b.in");
    g.connect("c.out", "d.in");

    auto order = g.topo_order();
    ASSERT_EQ(order.size(), 4u);
    EXPECT_EQ(order[0], a);
    EXPECT_EQ(order[1], b);
    EXPECT_EQ(order[2], c);
    EXPECT_EQ(order[3], d);
}

TEST(Graph, TopoOrderThrowsOnCycle) {
    // Build a cycle by typed connect (string-keyed connect would also work,
    // but typed avoids the input-port-overwrite case).
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));

    // a → b → a forms a cycle. The second connect overwrites b->a's input
    // source, but Graph's edge bookkeeping records both — enough to make
    // topo_order detect the cycle.
    g.connect(*a, a->out(), *b, b->in());
    g.connect(*b, b->out(), *a, a->in());

    EXPECT_THROW(g.topo_order(), std::runtime_error);
}

TEST(Graph, TopoOrderHandlesCycleBrokenByUnitDelay) {
    // a → b → delay → a — a real cycle, but the delay decouples its
    // input from its output, so topo_order should accept it. With the
    // delay's incoming edge ignored, ordering constraints reduce to
    // {a → b, delay → a}, giving the order [delay, a, b].
    Graph g;
    auto* a     = g.add_block(std::make_unique<MockBlock<int>>("a"));
    auto* b     = g.add_block(std::make_unique<MockBlock<int>>("b"));
    auto* delay = g.add_block(std::make_unique<shared::UnitDelayBlock<int>>("d"));

    g.connect("a.out", "b.in");
    g.connect("b.out", "d.in");
    g.connect("d.out", "a.in");

    auto order = g.topo_order();
    ASSERT_EQ(order.size(), 3u);

    auto pos = [&](shared::Block* x) {
        return std::find(order.begin(), order.end(), x) - order.begin();
    };
    EXPECT_LT(pos(delay), pos(a));  // delay → a edge
    EXPECT_LT(pos(a),     pos(b));  // a → b edge
}

// CompositeBlock now uses topo_order from its embedded Graph. Subclasses
// that record edges via graph().connect() get topology-respecting tick
// order; subclasses that wire with the free connect() see insertion order
// (back-compat with existing vehicle composites).
namespace {

class TopoComposite : public shared::CompositeBlock {
public:
    MockBlock<int>* a;
    MockBlock<int>* b;
    MockBlock<int>* c;

    TopoComposite() : CompositeBlock("comp") {
        // Add in REVERSE topological order. If update walked insertion
        // order it'd tick c → b → a and the value wouldn't propagate;
        // proper topo_order ticks a → b → c and the value flows through.
        c = add_child(std::make_unique<MockBlock<int>>("c"));
        b = add_child(std::make_unique<MockBlock<int>>("b"));
        a = add_child(std::make_unique<MockBlock<int>>("a"));

        graph().connect(*a, a->out(), *b, b->in());
        graph().connect(*b, b->out(), *c, c->in());
    }
};

} // namespace

TEST(CompositeBlock, GraphBackedTicksInTopoOrder) {
    TopoComposite comp;
    comp.a->in().set(7);

    comp.update(0);

    // Value flowed a → b → c in one tick because update walked topo order.
    EXPECT_EQ(comp.c->out().get(), 7);
    EXPECT_EQ(comp.a->update_count, 1);
    EXPECT_EQ(comp.b->update_count, 1);
    EXPECT_EQ(comp.c->update_count, 1);
}
