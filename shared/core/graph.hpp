#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "block.hpp"
#include "misuse.hpp"

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
    // Move ctor/assign aren't auto-declared once copy ctor is `= delete`,
    // but they're safe — vector + unordered_multimap members are movable
    // and pointer identity is preserved (unique_ptrs move ownership without
    // touching the heap object).
    Graph(Graph&&)            = default;
    Graph& operator=(Graph&&) = default;

    // Take ownership of `b` and register it under its declared name. Returns
    // a non-owning typed pointer the caller can use for wiring. Throws
    // std::invalid_argument if a block with the same name is already in the
    // graph — names are the canonical handles for graph-spec wiring later,
    // so duplicates are a misconfig, not a soft-fail.
    template<typename T>
    T* add_block(std::unique_ptr<T> b) {
        // Upcast to Block* before calling name(): T may multi-inherit
        // (e.g. SensorBlock combines TypedBlock with sensors::Sensor<>,
        // both of which expose a `name()` method with different signatures
        // — direct `b->name()` would be ambiguous). The upcast itself is
        // unambiguous because only one Block lives in any block subclass's
        // hierarchy.
        Block* as_block = b.get();
        if (find(as_block->name()) != nullptr) {
            detail::invalid_argument(
                "Graph::add_block: duplicate name '" + as_block->name() + "'");
        }
        T* raw = b.get();
        blocks_.push_back(std::move(b));
        ++version_;
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
            detail::invalid_argument(
                "Graph::connect: block not in graph (call add_block first)");
        }
        ::shared::connect(out, in);
        const std::size_t idx = edges_.size();
        edges_.push_back(Edge{&src, out.name, &dst, in.name});
        outgoing_index_.emplace(&src, idx);
        incoming_index_.emplace(&dst, idx);
        ++version_;
    }

    // String-keyed connect for graph specs. Path format is "block_name.port_name".
    // Throws std::invalid_argument on any failure (missing block, missing port,
    // type mismatch, direction mismatch, malformed path) — the underlying
    // wiring is not mutated and no edge is recorded on failure.
    void connect(std::string_view src_path, std::string_view dst_path) {
        const auto [src_block_name, src_port_name] = split_path(src_path);
        const auto [dst_block_name, dst_port_name] = split_path(dst_path);

        Block* src = find(src_block_name);
        if (!src) detail::invalid_argument(
            std::string("Graph::connect: source block '") +
            std::string(src_block_name) + "' not found");

        Block* dst = find(dst_block_name);
        if (!dst) detail::invalid_argument(
            std::string("Graph::connect: destination block '") +
            std::string(dst_block_name) + "' not found");

        IPort* src_port = src->port(src_port_name);
        if (!src_port) detail::invalid_argument(
            std::string("Graph::connect: port '") + std::string(src_port_name) +
            "' not found on block '" + std::string(src_block_name) + "'");

        IPort* dst_port = dst->port(dst_port_name);
        if (!dst_port) detail::invalid_argument(
            std::string("Graph::connect: port '") + std::string(dst_port_name) +
            "' not found on block '" + std::string(dst_block_name) + "'");

        if (src_port->is_input()) detail::invalid_argument(
            std::string("Graph::connect: source port '") +
            std::string(src_block_name) + "." + std::string(src_port_name) +
            "' is an input, not an output");

        // connect_to enforces the destination-is-input check and the type match,
        // and only mutates port state on success. Recording the edge is the
        // post-success bookkeeping step.
        src_port->connect_to(*dst_port);

        const std::size_t idx = edges_.size();
        edges_.push_back(Edge{src,
                              std::string(src_port_name),
                              dst,
                              std::string(dst_port_name)});
        outgoing_index_.emplace(src, idx);
        incoming_index_.emplace(dst, idx);
        ++version_;
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

    // Topologically order the blocks so each block appears AFTER all its
    // data sources. Tiebreak by insertion order — multiple valid orders
    // exist when blocks aren't connected; the earliest-inserted block
    // wins. Throws std::runtime_error on cycles that aren't broken by a
    // unit-delay block.
    //
    // Edges that terminate at a `is_delay()` block don't constrain the
    // order — the delay decouples its input from its output (input is
    // latched for next tick), so the cycle is broken at the delay's
    // input. Algebraic loops *not* broken by a delay still throw.
    std::vector<Block*> topo_order() const {
        std::vector<Block*> all;
        all.reserve(blocks_.size());
        for (const auto& b : blocks_) all.push_back(b.get());

        // Per-block remaining in-degree; mutated as we consume nodes.
        // Edges into delay blocks are excluded entirely (they don't
        // constrain ordering — see comment above).
        std::unordered_map<const Block*, int> remaining_in;
        for (auto* b : all) remaining_in[b] = 0;
        for (const auto& e : edges_) {
            if (e.dst_block->is_delay()) continue;
            remaining_in[e.dst_block]++;
        }

        std::vector<Block*> result;
        result.reserve(all.size());
        std::vector<bool> consumed(all.size(), false);

        while (result.size() < all.size()) {
            // Find the earliest-inserted block whose dependencies have all
            // been visited. Re-scanning from the start each iteration is
            // what gives us the deterministic insertion-order tiebreak.
            bool progress = false;
            for (std::size_t i = 0; i < all.size(); ++i) {
                if (consumed[i]) continue;
                if (remaining_in[all[i]] != 0) continue;

                Block* picked = all[i];
                result.push_back(picked);
                consumed[i] = true;

                // Unlock successors by decrementing their in-degree —
                // skipping the delay-terminated edges that we never
                // counted in the first place.
                auto range = outgoing_index_.equal_range(picked);
                for (auto it = range.first; it != range.second; ++it) {
                    Block* successor = edges_[it->second].dst_block;
                    if (successor->is_delay()) continue;
                    remaining_in[successor]--;
                }
                progress = true;
                break;
            }
            if (!progress) {
                detail::runtime_error(
                    "Graph::topo_order: cycle detected without a unit-delay "
                    "on the back-edge (visited " +
                    std::to_string(result.size()) + " of " +
                    std::to_string(all.size()) + " blocks)");
            }
        }
        return result;
    }

    std::size_t block_count() const { return blocks_.size(); }
    std::size_t edge_count()  const { return edges_.size(); }

    // Monotonic counter incremented on every mutation (add_block, connect).
    // Lets external observers (CompositeBlock's topo cache, future signal-
    // tracer subscriptions) detect when their cached view is stale without
    // re-running the full topo each tick. Read-only operations don't bump it.
    std::size_t version() const { return version_; }

    const std::vector<Edge>&                   edges()  const { return edges_; }
    const std::vector<std::unique_ptr<Block>>& blocks() const { return blocks_; }

private:
    bool owns(const Block* b) const {
        for (const auto& p : blocks_) if (p.get() == b) return true;
        return false;
    }

    // Split "block.port" into its two parts. Throws if there's no '.'.
    static std::pair<std::string_view, std::string_view>
    split_path(std::string_view path) {
        const auto dot = path.find('.');
        if (dot == std::string_view::npos) {
            detail::invalid_argument(
                std::string("Graph::connect: path '") +
                std::string(path) + "' missing '.' separator");
        }
        return {path.substr(0, dot), path.substr(dot + 1)};
    }

    std::vector<std::unique_ptr<Block>>                blocks_;
    std::vector<Edge>                                   edges_;
    std::unordered_multimap<const Block*, std::size_t>  outgoing_index_;
    std::unordered_multimap<const Block*, std::size_t>  incoming_index_;
    std::size_t                                         version_ = 0;
};

// A block that owns and schedules a set of child blocks.
//
// **Safety contract:** the topology is frozen before any tick. Subclass
// constructors must call `freeze()` after they're done adding children
// and registering edges (or after the wiring has been loaded). `update()`
// then walks a pre-computed `cached_order_` with zero allocation. This is
// what makes the runtime safe to use on the embedded hot path.
//
// `update()` aborts if:
//   - freeze() was never called (subclass forgot it);
//   - the graph was mutated after freeze() without a re-freeze (caller
//     bumped Graph::version() via add_block/connect post-freeze).
//
// A graph misconfig at runtime is always a programmer error — the abort is
// the right behaviour: better than silently recomputing inside the tick.
class CompositeBlock : public Block {
public:
    explicit CompositeBlock(const std::string& name, uint32_t update_period_us = 0)
        : Block(name, update_period_us) {}

    // Build the topological execution order for the current graph state.
    // Must be called by the subclass ctor (or by user setup code) after
    // all children/edges have been added, and before the first update().
    // Calling freeze() again is fine — it refreshes the cache; useful when
    // graph mutations are explicitly batched between ticks.
    void freeze() {
        cached_order_   = graph_.topo_order();    // throws on cycle: cache untouched
        cached_version_ = graph_.version();
    }

    bool update(uint64_t t) override {
        if (cached_version_ == kInvalidVersion) {
            detail::runtime_error(
                "CompositeBlock::update: graph not frozen — call freeze() "
                "after construction (block '" + name() + "')");
        }
        if (cached_version_ != graph_.version()) {
            detail::runtime_error(
                "CompositeBlock::update: graph mutated after freeze() — "
                "call freeze() again (block '" + name() + "')");
        }
        for (auto* child : cached_order_) {
            if (child->is_due(t)) child->update(t);
        }
        mark_updated(t);
        return true;
    }

    // Subclasses can register edges (graph().connect(...)) so topo_order
    // respects data-flow ordering. Const overload for read-only introspection.
    Graph&       graph()       { return graph_; }
    const Graph& graph() const { return graph_; }

protected:
    template<typename T>
    T* add_child(std::unique_ptr<T> block) {
        return graph_.add_block(std::move(block));
    }

private:
    static constexpr std::size_t kInvalidVersion = static_cast<std::size_t>(-1);

    Graph                graph_;
    std::vector<Block*>  cached_order_;
    std::size_t          cached_version_ = kInvalidVersion;
};

} // namespace shared
