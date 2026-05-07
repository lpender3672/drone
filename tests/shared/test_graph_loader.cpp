#include <gtest/gtest.h>
#include <stdexcept>
#include "core/graph.hpp"
#include "core/graph_loader.hpp"
#include "mocks/mock_block.h"

using shared::BlockFactory;
using shared::BlockParams;
using shared::Graph;
using shared::load_graph_json;
using mocks::MockBlock;

namespace {

// Factory ctor reused across tests: produces a MockBlock<double> and
// stamps "seed_value" onto its input port if provided.
auto mock_double_ctor() {
    return [](const std::string& name, const BlockParams& params) -> std::unique_ptr<shared::Block> {
        auto b = std::make_unique<MockBlock<double>>(name);
        if (params.has("seed_value")) {
            b->in().set(params.get_double("seed_value"));
        }
        return b;
    };
}

BlockFactory make_factory_with_mock() {
    BlockFactory f;
    f.register_type("mock", mock_double_ctor());
    return f;
}

} // namespace

TEST(GraphLoader, EmptyDocumentProducesEmptyGraph) {
    auto f = make_factory_with_mock();
    Graph g;

    load_graph_json("{}", f, g);

    EXPECT_EQ(g.block_count(), 0u);
    EXPECT_EQ(g.edge_count(),  0u);
}

TEST(GraphLoader, EmptyBlocksAndEdgesArrays) {
    auto f = make_factory_with_mock();
    Graph g;

    load_graph_json(R"({"blocks": [], "edges": []})", f, g);

    EXPECT_EQ(g.block_count(), 0u);
    EXPECT_EQ(g.edge_count(),  0u);
}

TEST(GraphLoader, LoadsSingleBlock) {
    auto f = make_factory_with_mock();
    Graph g;

    load_graph_json(R"({
        "blocks": [{"type": "mock", "name": "alpha"}]
    })", f, g);

    ASSERT_EQ(g.block_count(), 1u);
    auto* b = g.find("alpha");
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(b->name(), "alpha");
}

TEST(GraphLoader, LoadsMultipleBlocksInOrder) {
    auto f = make_factory_with_mock();
    Graph g;

    load_graph_json(R"({
        "blocks": [
            {"type": "mock", "name": "a"},
            {"type": "mock", "name": "b"},
            {"type": "mock", "name": "c"}
        ]
    })", f, g);

    EXPECT_EQ(g.block_count(), 3u);
    // Insertion order matters for topo tiebreak; verify it's preserved.
    EXPECT_EQ(g.blocks()[0]->name(), "a");
    EXPECT_EQ(g.blocks()[1]->name(), "b");
    EXPECT_EQ(g.blocks()[2]->name(), "c");
}

TEST(GraphLoader, LoadsEdges) {
    auto f = make_factory_with_mock();
    Graph g;

    load_graph_json(R"({
        "blocks": [
            {"type": "mock", "name": "a"},
            {"type": "mock", "name": "b"}
        ],
        "edges": [
            {"from": "a.out", "to": "b.in"}
        ]
    })", f, g);

    ASSERT_EQ(g.edge_count(), 1u);
    const auto& e = g.edges().front();
    EXPECT_EQ(e.src_block->name(), "a");
    EXPECT_EQ(e.dst_block->name(), "b");
    EXPECT_EQ(e.src_port, "out");
    EXPECT_EQ(e.dst_port, "in");
}

TEST(GraphLoader, ParamsForwardToFactory) {
    auto f = make_factory_with_mock();
    Graph g;

    load_graph_json(R"({
        "blocks": [
            {"type": "mock", "name": "seeded", "params": {"seed_value": 17.5}}
        ]
    })", f, g);

    auto* mock = dynamic_cast<MockBlock<double>*>(g.find("seeded"));
    ASSERT_NE(mock, nullptr);
    EXPECT_DOUBLE_EQ(mock->in().get(), 17.5);
}

TEST(GraphLoader, ThrowsOnMalformedJson) {
    auto f = make_factory_with_mock();
    Graph g;

    EXPECT_THROW(load_graph_json("{not json", f, g), std::invalid_argument);
}

TEST(GraphLoader, ThrowsOnTopLevelArray) {
    auto f = make_factory_with_mock();
    Graph g;

    // Top-level must be an object; an array isn't a valid graph spec.
    EXPECT_THROW(load_graph_json("[]", f, g), std::invalid_argument);
}

TEST(GraphLoader, ThrowsOnUnknownBlockType) {
    BlockFactory f;  // no types registered
    Graph g;

    EXPECT_THROW(load_graph_json(R"({
        "blocks": [{"type": "ghost", "name": "x"}]
    })", f, g), std::invalid_argument);
}

TEST(GraphLoader, ThrowsOnMissingBlockTypeField) {
    auto f = make_factory_with_mock();
    Graph g;

    EXPECT_THROW(load_graph_json(R"({
        "blocks": [{"name": "x"}]
    })", f, g), std::invalid_argument);
}

TEST(GraphLoader, ThrowsOnMissingBlockNameField) {
    auto f = make_factory_with_mock();
    Graph g;

    EXPECT_THROW(load_graph_json(R"({
        "blocks": [{"type": "mock"}]
    })", f, g), std::invalid_argument);
}

TEST(GraphLoader, ThrowsOnMissingEdgeFields) {
    auto f = make_factory_with_mock();
    Graph g;

    EXPECT_THROW(load_graph_json(R"({
        "blocks": [{"type": "mock", "name": "a"}, {"type": "mock", "name": "b"}],
        "edges":  [{"from": "a.out"}]
    })", f, g), std::invalid_argument);
}

TEST(GraphLoader, ThrowsOnUnsupportedParamType) {
    auto f = make_factory_with_mock();
    Graph g;

    // BlockParams supports number, bool, and string. Anything else (array,
    // nested object, null) is rejected explicitly rather than silently
    // dropped — better to fail fast than ship a misconfigured graph.
    EXPECT_THROW(load_graph_json(R"({
        "blocks": [{"type": "mock", "name": "x",
                    "params": {"seed_value": [1, 2, 3]}}]
    })", f, g), std::invalid_argument);

    EXPECT_THROW(load_graph_json(R"({
        "blocks": [{"type": "mock", "name": "y",
                    "params": {"nested": {"foo": 1}}}]
    })", f, g), std::invalid_argument);
}

// Integration: load a JSON spec, drop into a CompositeBlock, freeze, tick,
// verify zero-copy dataflow propagates through the loaded topology.
TEST(GraphLoader, IntegrationLoadAndTick) {
    class JsonComposite : public shared::CompositeBlock {
    public:
        JsonComposite() : CompositeBlock("from_json") {}
    };

    auto f = make_factory_with_mock();
    JsonComposite comp;

    load_graph_json(R"({
        "blocks": [
            {"type": "mock", "name": "a", "params": {"seed_value": 23.0}},
            {"type": "mock", "name": "b"}
        ],
        "edges": [
            {"from": "a.out", "to": "b.in"}
        ]
    })", f, comp.graph());

    comp.freeze();
    comp.update(0);

    auto* b = dynamic_cast<MockBlock<double>*>(comp.graph().find("b"));
    ASSERT_NE(b, nullptr);
    EXPECT_DOUBLE_EQ(b->out().get(), 23.0);
}
