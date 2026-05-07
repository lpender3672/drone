#pragma once

#include <cstddef>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <core/block.hpp>
#include <core/graph.hpp>

namespace editor {

// Maps a `shared::Graph` onto integer IDs that ImNodes consumes.
//
// ImNodes addresses everything by `int` — node IDs, pin IDs, link IDs.
// Graph addresses everything by pointer + name. GraphLayout walks the
// graph once (`rebuild`) and assigns sequential IDs from 1 (ImNodes
// treats 0 as "no selection" in some APIs, so live IDs start at 1):
//
//   node ids:   one per Block, in `Graph::blocks()` insertion order
//   pin  ids:   one per IPort (input + output), in `Block::ports()` order
//   link ids:   one per `Graph::edges()` entry, in insertion order
//
// All IDs share a single counter so node / pin / link IDs are mutually
// distinct — ImNodes can mistakenly accept a pin ID where a node ID
// is expected; making the namespaces disjoint catches that as a hard
// lookup miss instead of a silent rendering bug.
//
// Read-only as of slice 2: `rebuild()` is idempotent on an unchanged
// graph and clears all state on each call. When live editing lands
// (slice 5) we'll need ID stability across mutations — this class is
// the natural place to put that policy.
class GraphLayout {
public:
    struct PinInfo {
        int                  id;
        const shared::IPort* port;
        bool                 is_input;
    };

    struct LinkInfo {
        int id;
        int src_pin;
        int dst_pin;
    };

    void rebuild(const shared::Graph& graph) {
        clear();

        // Stable ordering: blocks in insertion order, ports in registration
        // order. Both are deterministic functions of the input graph so the
        // RebuildOnUnchangedGraphYieldsSameIds invariant falls out naturally.
        for (const auto& block_uptr : graph.blocks()) {
            const shared::Block* block = block_uptr.get();
            const int node_id = next_id_++;
            block_to_node_[block] = node_id;
            node_to_block_[node_id] = block;

            std::vector<PinInfo>& pins = pins_by_block_[block];
            for (const shared::IPort* port : block->ports()) {
                const int pin_id = next_id_++;
                port_to_pin_[port] = pin_id;
                pin_to_port_[pin_id] = port;
                pins.push_back(PinInfo{pin_id, port, port->is_input()});
            }
        }

        for (const shared::Edge& e : graph.edges()) {
            const shared::IPort* src_port = e.src_block->port(e.src_port);
            const shared::IPort* dst_port = e.dst_block->port(e.dst_port);
            // The graph guarantees these resolve — connect() rejects edges
            // referencing missing ports — so this is just defensive routing.
            if (!src_port || !dst_port) continue;

            const int link_id = next_id_++;
            links_.push_back(LinkInfo{
                link_id,
                port_to_pin_.at(src_port),
                port_to_pin_.at(dst_port)
            });
        }
    }

    int node_id(const shared::Block& block) const {
        const auto it = block_to_node_.find(&block);
        if (it == block_to_node_.end()) {
            throw std::invalid_argument(
                "GraphLayout::node_id: block '" + block.name() + "' not in layout");
        }
        return it->second;
    }

    int pin_id(const shared::IPort& port) const {
        const auto it = port_to_pin_.find(&port);
        if (it == port_to_pin_.end()) {
            throw std::invalid_argument(
                "GraphLayout::pin_id: port not in layout");
        }
        return it->second;
    }

    // Reverse lookups — return nullptr (not throw) on miss. The canvas
    // will hand us IDs from ImNodes that may correspond to entries from
    // before a rebuild; treating an unknown ID as a soft miss keeps the
    // render path simple.
    const shared::Block* block_for_node(int node_id) const {
        const auto it = node_to_block_.find(node_id);
        return it == node_to_block_.end() ? nullptr : it->second;
    }
    const shared::IPort* port_for_pin(int pin_id) const {
        const auto it = pin_to_port_.find(pin_id);
        return it == pin_to_port_.end() ? nullptr : it->second;
    }

    std::vector<PinInfo> pins_of(const shared::Block& block) const {
        const auto it = pins_by_block_.find(&block);
        return it == pins_by_block_.end() ? std::vector<PinInfo>{} : it->second;
    }

    const std::vector<LinkInfo>& links() const { return links_; }

    std::size_t node_count() const { return block_to_node_.size(); }
    std::size_t link_count() const { return links_.size(); }

private:
    void clear() {
        block_to_node_.clear();
        node_to_block_.clear();
        port_to_pin_.clear();
        pin_to_port_.clear();
        pins_by_block_.clear();
        links_.clear();
        next_id_ = 1;  // ImNodes treats 0 as a sentinel — start at 1.
    }

    int next_id_ = 1;

    std::unordered_map<const shared::Block*, int>          block_to_node_;
    std::unordered_map<int, const shared::Block*>          node_to_block_;
    std::unordered_map<const shared::IPort*, int>          port_to_pin_;
    std::unordered_map<int, const shared::IPort*>          pin_to_port_;
    std::unordered_map<const shared::Block*, std::vector<PinInfo>> pins_by_block_;
    std::vector<LinkInfo>                                   links_;
};

} // namespace editor
