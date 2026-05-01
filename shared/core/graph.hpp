#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "block.hpp"

namespace shared {

// First-class edge between two ports of two blocks owned by a Graph.
// String port names because Tier 2's YAML graph specs key on names; block
// pointer for stable identity (the Graph owns the unique_ptr).
struct Edge {
    Block*      src_block;
    std::string src_port;
    Block*      dst_block;
    std::string dst_port;
};

// Owns a set of blocks and tracks their wiring as data — the foundation
// for graph-driven composition (Tier 2 of docs/REQUIREMENTS.md). Slice 1
// only covers ownership + edge recording with typed-port `connect()`;
// type erasure, factory, topo sort, and YAML loading are subsequent slices.
class Graph {
public:
    Graph() = default;
    Graph(const Graph&)            = delete;
    Graph& operator=(const Graph&) = delete;

    // Take ownership of `b` and register it under its declared name. Returns
    // a non-owning typed pointer the caller can use for wiring. Throws
    // std::invalid_argument if a block with the same name is already in the
    // graph — names are the canonical handles for graph-spec wiring later,
    // so duplicates are a misconfig, not a soft-fail.
    template<typename T>
    T* add_block(std::unique_ptr<T> b) {
        if (find(b->name()) != nullptr) {
            throw std::invalid_argument("Graph::add_block: duplicate name '" + b->name() + "'");
        }
        T* raw = b.get();
        blocks_.push_back(std::move(b));
        return raw;
    }

    Block* find(std::string_view name) const {
        for (const auto& b : blocks_) {
            if (b->name() == name) return b.get();
        }
        return nullptr;
    }

    // Wire `out` → `in` and record the edge as graph metadata. Both blocks
    // must already have been registered via add_block(); a stray block
    // pointer is a misconfig and throws std::invalid_argument before any
    // pointer is wired (the underlying input port is unmodified on failure).
    template<typename T>
    void connect(Block& src, OutputPort<T>& out,
                 Block& dst, InputPort<T>& in) {
        if (!owns(&src) || !owns(&dst)) {
            throw std::invalid_argument(
                "Graph::connect: block not in graph (call add_block first)");
        }
        ::shared::connect(out, in);
        const std::size_t idx = edges_.size();
        edges_.push_back(Edge{&src, out.name, &dst, in.name});
        outgoing_index_.emplace(&src, idx);
        incoming_index_.emplace(&dst, idx);
    }

    std::vector<Edge> outgoing(const Block* src) const {
        std::vector<Edge> out;
        auto range = outgoing_index_.equal_range(src);
        for (auto it = range.first; it != range.second; ++it) {
            out.push_back(edges_[it->second]);
        }
        return out;
    }

    std::vector<Edge> incoming(const Block* dst) const {
        std::vector<Edge> in;
        auto range = incoming_index_.equal_range(dst);
        for (auto it = range.first; it != range.second; ++it) {
            in.push_back(edges_[it->second]);
        }
        return in;
    }

    std::size_t block_count() const { return blocks_.size(); }
    std::size_t edge_count()  const { return edges_.size(); }

    const std::vector<Edge>&                   edges()  const { return edges_; }
    const std::vector<std::unique_ptr<Block>>& blocks() const { return blocks_; }

private:
    bool owns(const Block* b) const {
        for (const auto& p : blocks_) if (p.get() == b) return true;
        return false;
    }

    std::vector<std::unique_ptr<Block>>                blocks_;
    std::vector<Edge>                                   edges_;
    std::unordered_multimap<const Block*, std::size_t>  outgoing_index_;
    std::unordered_multimap<const Block*, std::size_t>  incoming_index_;
};

} // namespace shared
