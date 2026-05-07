#pragma once

#include <nlohmann/json.hpp>
#include <string>

#include "block_factory.hpp"
#include "graph.hpp"
#include "misuse.hpp"
#include "tracer.hpp"

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
//         "params": { "<key>": <number|bool|string>, ... }  // optional
//       },
//       ...
//     ],
//     "edges": [
//       { "from": "<block_name>.<port_name>",
//         "to":   "<block_name>.<port_name>" },
//       ...
//     ],
//     "traces": [   // optional; populated only by load_traces_json()
//       { "port": "<block_name>.<port_name>",
//         "name": "<csv_column_prefix>" },
//       ...
//     ]
//   }
//
// `blocks`, `edges`, and `traces` are each optional — an empty object is
// a valid (empty) graph with no traces. Order in `blocks` becomes
// insertion order in the graph (deterministic topo tiebreak); order in
// `traces` becomes column order in the output CSV.

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
                    } else if (it.value().is_boolean()) {
                        params.set(it.key(), it.value().get<bool>() ? 1.0 : 0.0);
                    } else if (it.value().is_string()) {
                        params.set_string(it.key(), it.value().get<std::string>());
                    } else {
                        detail::invalid_argument(
                            "load_graph_json: param '" + it.key() + "' on block '" +
                            block_name + "' must be a number, bool, or string");
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

// Populate `tracer` from the document's `traces` array. The graph must
// already be loaded — `add_trace` looks ports up by name. Missing
// `traces` is fine (no-op); a malformed entry throws.
//
// The tracer must already have serializers registered for every port
// type referenced — typically via `register_quadrotor_traces(tracer)`
// or equivalent before this call.
inline void load_traces_json(const std::string& json_text,
                             const Graph&       graph,
                             SignalTracer&      tracer) {
    using nlohmann::json;

    json doc;
    try {
        doc = json::parse(json_text);
    } catch (const json::parse_error& e) {
        detail::invalid_argument(std::string("load_traces_json: malformed JSON: ") + e.what());
    }

    if (!doc.is_object()) {
        detail::invalid_argument("load_traces_json: top-level value must be an object");
    }

    if (!doc.contains("traces")) return;

    const auto& traces_node = doc["traces"];
    if (!traces_node.is_array()) {
        detail::invalid_argument("load_traces_json: 'traces' must be an array");
    }
    for (const auto& trace_spec : traces_node) {
        if (!trace_spec.contains("port") || !trace_spec["port"].is_string()) {
            detail::invalid_argument("load_traces_json: each trace must have a string 'port'");
        }
        if (!trace_spec.contains("name") || !trace_spec["name"].is_string()) {
            detail::invalid_argument("load_traces_json: each trace must have a string 'name'");
        }
        tracer.add_trace(graph,
                         trace_spec["port"].get<std::string>(),
                         trace_spec["name"].get<std::string>());
    }
}

} // namespace shared
