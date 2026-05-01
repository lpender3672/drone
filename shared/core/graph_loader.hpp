#pragma once

#include <nlohmann/json.hpp>
#include <string>

#include "block_factory.hpp"
#include "graph.hpp"
#include "misuse.hpp"

// Host-only utility: load a graph topology from a JSON document.
//
// The loader is the bridge between Tier 2's graph-driven composition and
// the existing block runtime — it walks the JSON, calls `factory.create()`
// per block (which the factory routes to the registered ctor), and calls
// `graph.connect()` per edge. Validation failures route through misuse.hpp,
// so on host they throw and on a notional embedded host (host-side YAML
// converter, etc.) they abort.
//
// **Not for embedded firmware**: this header pulls in nlohmann/json, which
// is host-only by design. Embedded targets construct their composites in
// C++ (the existing main.cpp pattern); the JSON path exists for sim and
// the eventual visual editor.
//
// JSON schema:
//   {
//     "blocks": [
//       { "type": "<registered_type_name>",
//         "name": "<unique_block_name>",
//         "params": { "<key>": <number>, ... }   // optional; omitted = empty
//       },
//       ...
//     ],
//     "edges": [
//       { "from": "<block_name>.<port_name>",
//         "to":   "<block_name>.<port_name>" },
//       ...
//     ]
//   }
//
// Both `blocks` and `edges` are optional — an empty object is a valid
// (empty) graph. Order in `blocks` becomes insertion order in the graph,
// which matters for deterministic topo tiebreak.

namespace shared {

inline void load_graph_json(const std::string& json_text,
                            const BlockFactory& factory,
                            Graph& graph) {
    using nlohmann::json;

    json doc;
    try {
        doc = json::parse(json_text);
    } catch (const json::parse_error& e) {
        detail::invalid_argument(std::string("load_graph_json: malformed JSON: ") + e.what());
    }

    if (!doc.is_object()) {
        detail::invalid_argument("load_graph_json: top-level value must be an object");
    }

    if (doc.contains("blocks")) {
        const auto& blocks_node = doc["blocks"];
        if (!blocks_node.is_array()) {
            detail::invalid_argument("load_graph_json: 'blocks' must be an array");
        }
        for (const auto& block_spec : blocks_node) {
            if (!block_spec.contains("type") || !block_spec["type"].is_string()) {
                detail::invalid_argument("load_graph_json: each block must have a string 'type'");
            }
            if (!block_spec.contains("name") || !block_spec["name"].is_string()) {
                detail::invalid_argument("load_graph_json: each block must have a string 'name'");
            }

            const std::string type_name  = block_spec["type"].get<std::string>();
            const std::string block_name = block_spec["name"].get<std::string>();

            BlockParams params;
            if (block_spec.contains("params")) {
                const auto& params_node = block_spec["params"];
                if (!params_node.is_object()) {
                    detail::invalid_argument(
                        "load_graph_json: 'params' for block '" + block_name +
                        "' must be an object");
                }
                for (auto it = params_node.begin(); it != params_node.end(); ++it) {
                    if (it.value().is_number()) {
                        params.set(it.key(), it.value().get<double>());
                    } else {
                        detail::invalid_argument(
                            "load_graph_json: param '" + it.key() + "' on block '" +
                            block_name + "' must be a number (slice 7 supports doubles only)");
                    }
                }
            }

            // factory.create() throws on unknown type; graph.add_block() throws
            // on duplicate name. Both propagate up via misuse.
            graph.add_block(factory.create(type_name, block_name, params));
        }
    }

    if (doc.contains("edges")) {
        const auto& edges_node = doc["edges"];
        if (!edges_node.is_array()) {
            detail::invalid_argument("load_graph_json: 'edges' must be an array");
        }
        for (const auto& edge_spec : edges_node) {
            if (!edge_spec.contains("from") || !edge_spec["from"].is_string()) {
                detail::invalid_argument("load_graph_json: each edge must have a string 'from'");
            }
            if (!edge_spec.contains("to") || !edge_spec["to"].is_string()) {
                detail::invalid_argument("load_graph_json: each edge must have a string 'to'");
            }
            graph.connect(edge_spec["from"].get<std::string>(),
                          edge_spec["to"].get<std::string>());
        }
    }
}

} // namespace shared
