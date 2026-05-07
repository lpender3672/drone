#pragma once

#include <imgui.h>
#include <imnodes.h>

#include <cstddef>
#include <string>

#include <core/graph.hpp>

#include "graph_layout.hpp"

namespace editor {

// Renders a `shared::Graph` onto an ImNodes canvas. Slice 2: read-only.
// Pan + zoom + node drag come for free from ImNodes; we don't write any
// of that ourselves. Edge creation, deletion, and palette drag-to-create
// land in slice 5; live signal plotting in slice 6.
//
// Lifecycle:
//   1. set_graph(g) once after a JSON load (or whenever the graph
//      identity changes). Internally calls layout_.rebuild(*g) and
//      seeds initial node positions.
//   2. render() per frame inside the editor's main render loop.
//
// Position seeding: ImNodes stores node screen-space positions
// internally, so we only place them once — on the first frame after
// each `set_graph()` — using a deterministic grid keyed off the layout's
// node IDs. The user can then drag nodes; ImNodes preserves drags.
class CanvasView {
public:
    void set_graph(const shared::Graph* graph) {
        graph_ = graph;
        if (graph_) layout_.rebuild(*graph_);
        positions_pending_ = (graph_ != nullptr);
    }

    const GraphLayout& layout() const { return layout_; }

    // Call inside an ImGui Begin/End scope. Renders the editor canvas
    // filling the current ImGui window.
    void render() {
        ImNodes::BeginNodeEditor();

        if (graph_) {
            for (const auto& block_uptr : graph_->blocks()) {
                const shared::Block* block = block_uptr.get();
                const int node_id = layout_.node_id(*block);

                if (positions_pending_) {
                    ImNodes::SetNodeGridSpacePos(
                        node_id, grid_position_for(node_id));
                }

                ImNodes::BeginNode(node_id);

                ImNodes::BeginNodeTitleBar();
                ImGui::TextUnformatted(block->name().c_str());
                ImNodes::EndNodeTitleBar();

                for (const auto& pin : layout_.pins_of(*block)) {
                    if (pin.is_input) {
                        ImNodes::BeginInputAttribute(pin.id);
                        ImGui::TextUnformatted(
                            std::string(pin.port->port_name()).c_str());
                        ImNodes::EndInputAttribute();
                    } else {
                        ImNodes::BeginOutputAttribute(pin.id);
                        // Right-align the output pin label by reserving
                        // a fixed column width — keeps the canvas tidy
                        // when ports have asymmetric name lengths
                        // (e.g. "imu" vs "disturbance_torque").
                        const float text_width = ImGui::CalcTextSize(
                            std::string(pin.port->port_name()).c_str()).x;
                        ImGui::Indent(kNodeContentWidth - text_width);
                        ImGui::TextUnformatted(
                            std::string(pin.port->port_name()).c_str());
                        ImGui::Unindent(kNodeContentWidth - text_width);
                        ImNodes::EndOutputAttribute();
                    }
                }

                ImNodes::EndNode();
            }

            for (const auto& link : layout_.links()) {
                ImNodes::Link(link.id, link.src_pin, link.dst_pin);
            }
        }

        ImNodes::EndNodeEditor();
        positions_pending_ = false;
    }

private:
    // Width budget for a node's content column. Used to right-align
    // output pin labels. Picked to comfortably fit "disturbance_torque"
    // at default ImGui font size; still readable for shorter labels.
    static constexpr float kNodeContentWidth = 140.0f;

    // Deterministic initial grid: 4 columns, 200 px horizontal pitch,
    // 160 px vertical. Good enough that quad.json's 11 nodes don't all
    // stack on top of each other; user can drag from there.
    ImVec2 grid_position_for(int node_id) const {
        constexpr int   cols     = 4;
        constexpr float dx       = 220.0f;
        constexpr float dy       = 170.0f;
        constexpr float origin_x = 40.0f;
        constexpr float origin_y = 60.0f;
        // node_id starts at 1 and counts blocks-then-pins, so derive
        // the slot from the block's order index instead. We don't have
        // direct access to that here; re-derive from the layout.
        std::size_t slot = 0;
        for (const auto& b : graph_->blocks()) {
            if (layout_.node_id(*b) == node_id) break;
            ++slot;
        }
        const float col = static_cast<float>(slot % cols);
        const float row = static_cast<float>(slot / cols);
        return ImVec2(origin_x + col * dx, origin_y + row * dy);
    }

    const shared::Graph* graph_              = nullptr;
    GraphLayout          layout_;
    bool                 positions_pending_  = false;
};

} // namespace editor
