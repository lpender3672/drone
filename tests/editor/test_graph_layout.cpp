#include <gtest/gtest.h>

#include <core/graph.hpp>
#include <graph_layout.hpp>

#include "../mocks/mock_block.h"

using editor::GraphLayout;
using mocks::MockBlock;
using shared::Graph;

// GraphLayout is the pure ID-mapping layer between shared::Graph (a
// graph of blocks/ports/edges keyed by names + pointers) and ImNodes
// (which addresses everything by `int` IDs). Forward + reverse lookup
// for nodes (blocks), pins (ports), and links (edges). No ImGui
// dependency — the rendering layer (CanvasView) consumes a built layout.

namespace {

// Two MockBlock<>s wired output→input, with their ports retained.
struct TwoBlockGraph {
    Graph graph;
    MockBlock<>* a;
    MockBlock<>* b;

    TwoBlockGraph() {
        a = graph.add_block(std::make_unique<MockBlock<>>("a"));
        b = graph.add_block(std::make_unique<MockBlock<>>("b"));
        graph.connect(*a, a->out(), *b, b->in());
    }
};

} // namespace

// ---- empty + smallest cases -----------------------------------------------

TEST(GraphLayout, EmptyGraphHasNoNodesNoLinks) {
    Graph g;
    GraphLayout layout;
    layout.rebuild(g);
    EXPECT_EQ(layout.node_count(), 0u);
    EXPECT_EQ(layout.link_count(), 0u);
    EXPECT_TRUE(layout.links().empty());
}

TEST(GraphLayout, SingleBlockProducesOneNodeAndPinsForEachPort) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    GraphLayout layout;
    layout.rebuild(g);

    EXPECT_EQ(layout.node_count(), 1u);
    EXPECT_EQ(layout.link_count(), 0u);

    // MockBlock has two ports ("in", "out"); both should be in the layout.
    const auto pins = layout.pins_of(*a);
    ASSERT_EQ(pins.size(), 2u);

    int input_count  = 0;
    int output_count = 0;
    for (const auto& p : pins) {
        if (p.is_input) ++input_count; else ++output_count;
    }
    EXPECT_EQ(input_count,  1);
    EXPECT_EQ(output_count, 1);
}

// ---- ID assignment + round-trip lookups -----------------------------------

TEST(GraphLayout, NodeAndPortLookupsRoundTrip) {
    TwoBlockGraph t;
    GraphLayout layout;
    layout.rebuild(t.graph);

    const int a_id = layout.node_id(*t.a);
    const int b_id = layout.node_id(*t.b);
    EXPECT_NE(a_id, b_id);
    EXPECT_EQ(layout.block_for_node(a_id), static_cast<shared::Block*>(t.a));
    EXPECT_EQ(layout.block_for_node(b_id), static_cast<shared::Block*>(t.b));

    const int a_out_pin = layout.pin_id(t.a->out());
    const int b_in_pin  = layout.pin_id(t.b->in());
    EXPECT_NE(a_out_pin, b_in_pin);
    EXPECT_EQ(layout.port_for_pin(a_out_pin),
              static_cast<shared::IPort*>(&t.a->out()));
    EXPECT_EQ(layout.port_for_pin(b_in_pin),
              static_cast<shared::IPort*>(&t.b->in()));
}

TEST(GraphLayout, IdsArePositive) {
    // ImNodes treats 0 as "no selection" in some APIs, so all live IDs
    // must be ≥ 1. Cheap to enforce, prevents subtle interaction bugs.
    TwoBlockGraph t;
    GraphLayout layout;
    layout.rebuild(t.graph);

    EXPECT_GE(layout.node_id(*t.a), 1);
    EXPECT_GE(layout.node_id(*t.b), 1);
    EXPECT_GE(layout.pin_id(t.a->out()), 1);
    EXPECT_GE(layout.pin_id(t.b->in()),  1);
}

TEST(GraphLayout, MissingBlockLookupThrows) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    GraphLayout layout;
    layout.rebuild(g);

    // A block that's not in the rebuilt graph isn't known to the layout.
    MockBlock<> stranger("not_in_graph");
    EXPECT_THROW(layout.node_id(stranger),         std::invalid_argument);
    EXPECT_THROW(layout.pin_id(stranger.out()),    std::invalid_argument);

    // Reverse lookups for unknown IDs return nullptr (querying the canvas
    // for a stale ID after a rebuild shouldn't throw — it's a soft miss).
    EXPECT_EQ(layout.block_for_node(99999), nullptr);
    EXPECT_EQ(layout.port_for_pin(99999),   nullptr);

    (void)a;
}

// ---- links ----------------------------------------------------------------

TEST(GraphLayout, LinkConnectsExpectedPins) {
    TwoBlockGraph t;
    GraphLayout layout;
    layout.rebuild(t.graph);

    ASSERT_EQ(layout.link_count(), 1u);
    const auto& link = layout.links()[0];
    EXPECT_EQ(link.src_pin, layout.pin_id(t.a->out()));
    EXPECT_EQ(link.dst_pin, layout.pin_id(t.b->in()));
    EXPECT_GE(link.id, 1);
}

TEST(GraphLayout, FanOutFromSamePortProducesMultipleLinks) {
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    auto* c = g.add_block(std::make_unique<MockBlock<>>("c"));
    g.connect(*a, a->out(), *b, b->in());
    g.connect(*a, a->out(), *c, c->in());

    GraphLayout layout;
    layout.rebuild(g);

    ASSERT_EQ(layout.link_count(), 2u);
    const int a_out = layout.pin_id(a->out());
    EXPECT_EQ(layout.links()[0].src_pin, a_out);
    EXPECT_EQ(layout.links()[1].src_pin, a_out);
    // Distinct destination pins.
    EXPECT_NE(layout.links()[0].dst_pin, layout.links()[1].dst_pin);
    // Distinct link IDs.
    EXPECT_NE(layout.links()[0].id, layout.links()[1].id);
}

// ---- rebuild stability ---------------------------------------------------

TEST(GraphLayout, RebuildOnUnchangedGraphYieldsSameIds) {
    // The simplest stability guarantee: the layout is a deterministic
    // function of the graph. Future slices may relax this for live
    // editing (preserve IDs across mutations), but for slice 2's read-
    // only canvas, idempotent rebuild() is the contract.
    TwoBlockGraph t;
    GraphLayout layout;
    layout.rebuild(t.graph);

    const int a_id_before     = layout.node_id(*t.a);
    const int a_out_pin_before = layout.pin_id(t.a->out());
    const int link_id_before  = layout.links()[0].id;

    layout.rebuild(t.graph);

    EXPECT_EQ(layout.node_id(*t.a),       a_id_before);
    EXPECT_EQ(layout.pin_id(t.a->out()),  a_out_pin_before);
    ASSERT_EQ(layout.link_count(), 1u);
    EXPECT_EQ(layout.links()[0].id, link_id_before);
}

TEST(GraphLayout, RebuildClearsStaleEntries) {
    TwoBlockGraph t;
    GraphLayout layout;
    layout.rebuild(t.graph);

    const int stale_a_id = layout.node_id(*t.a);

    // Rebuild from an empty graph — all prior entries must be gone.
    Graph empty;
    layout.rebuild(empty);
    EXPECT_EQ(layout.node_count(), 0u);
    EXPECT_EQ(layout.link_count(), 0u);
    EXPECT_EQ(layout.block_for_node(stale_a_id), nullptr);
}
