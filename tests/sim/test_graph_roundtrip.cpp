#include <gtest/gtest.h>

#include <fstream>
#include <set>
#include <sstream>
#include <string>

#include <core/block_factory.hpp>
#include <core/graph.hpp>
#include <core/graph_loader.hpp>
#include <quadcopter/registrations.hpp>

// Round-trip integration: load → dump → load → assert equivalence.
// Slice 4 contract: topology (blocks + edges) is preserved exactly;
// params fall back to compile-time defaults on the second load.
// This test locks the contract in CI so a regression in either
// direction (loader or dumper) shows up here, not in production
// editor save/load.

namespace {

std::string read_file(const std::string& path) {
    std::ifstream in(path);
    std::ostringstream ss; ss << in.rdbuf();
    return ss.str();
}

shared::BlockFactory make_quad_factory() {
    shared::BlockFactory f;
    sim::register_quadrotor_blocks(f);
    return f;
}

// Project an edge to a (from, to) string pair so two graphs can be
// compared as sets — edge ordering may differ across loads if the
// loader is ever changed (today it's stable, but contract-test against
// the set, not the order, to keep round-trip robust).
using EdgeKey = std::pair<std::string, std::string>;

std::set<EdgeKey> edge_keys(const shared::Graph& g) {
    std::set<EdgeKey> out;
    for (const auto& e : g.edges()) {
        out.emplace(e.src_block->name() + "." + e.src_port,
                    e.dst_block->name() + "." + e.dst_port);
    }
    return out;
}

std::set<std::string> block_names(const shared::Graph& g) {
    std::set<std::string> out;
    for (const auto& b : g.blocks()) out.insert(b->name());
    return out;
}

} // namespace

// CMake hands us the source-tree path so the test isn't sensitive to
// CWD (matters for IDE-launched runs).
#ifndef DRONE_REPO_ROOT
#define DRONE_REPO_ROOT "."
#endif

TEST(GraphRoundTrip, QuadJsonSurvivesLoadDumpLoad) {
    const std::string quad_path = std::string(DRONE_REPO_ROOT)
                                + "/sim/configs/quad.json";
    const std::string original = read_file(quad_path);
    ASSERT_FALSE(original.empty()) << "could not read " << quad_path;

    shared::BlockFactory factory = make_quad_factory();

    shared::Graph g1;
    ASSERT_NO_THROW(shared::load_graph_json(original, factory, g1));
    const std::size_t n_blocks = g1.block_count();
    const std::size_t n_edges  = g1.edge_count();
    ASSERT_GT(n_blocks, 0u);
    ASSERT_GT(n_edges,  0u);

    const std::string dumped = shared::dump_graph_json(g1);
    ASSERT_FALSE(dumped.empty());

    shared::Graph g2;
    ASSERT_NO_THROW(shared::load_graph_json(dumped, factory, g2));

    EXPECT_EQ(g2.block_count(), n_blocks);
    EXPECT_EQ(g2.edge_count(),  n_edges);
    EXPECT_EQ(block_names(g2),  block_names(g1));
    EXPECT_EQ(edge_keys(g2),    edge_keys(g1));
}

TEST(GraphRoundTrip, EkfCompareJsonSurvivesLoadDumpLoad) {
    // Same contract for the second canonical spec.
    const std::string ekf_path = std::string(DRONE_REPO_ROOT)
                               + "/sim/configs/ekf_compare.json";
    const std::string original = read_file(ekf_path);
    ASSERT_FALSE(original.empty());

    shared::BlockFactory factory = make_quad_factory();
    shared::Graph g1;
    shared::load_graph_json(original, factory, g1);
    const std::string dumped = shared::dump_graph_json(g1);
    shared::Graph g2;
    shared::load_graph_json(dumped, factory, g2);

    EXPECT_EQ(g2.block_count(), g1.block_count());
    EXPECT_EQ(g2.edge_count(),  g1.edge_count());
    EXPECT_EQ(block_names(g2),  block_names(g1));
    EXPECT_EQ(edge_keys(g2),    edge_keys(g1));
}
