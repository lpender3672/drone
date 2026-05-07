#pragma once

#include <functional>
#include <ostream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "block.hpp"
#include "graph.hpp"
#include "misuse.hpp"

// Signal-trace auto-logger. The Tier 2 replacement for the dead
// IInterBlockData / DataLogger infrastructure.
//
// Workflow:
//   1. The host registers a serializer per port type T it cares about
//      (Scalar, NavigationState, …) — one (HeaderFn, ValueFn) pair, keyed
//      by `detail::type_tag<T>::id()`.
//   2. The graph spec tags ports it wants logged. Each tagged port becomes
//      one or more CSV columns (the count is whatever the registered
//      header writer emits — Scalar = 1 column, NavigationState ≈ 13).
//   3. Each tick the host calls `write_row(out, time_us)`. The tracer
//      walks each trace, looks up the port's value via IPort::read_erased(),
//      and dispatches to the type's serializer.
//
// Host-only by design: depends on std::ostream which the embedded toolchain
// doesn't carry. Embedded logging continues via `ILogger`/SD on the raw
// sensor path (FR-6 in REQUIREMENTS.md).

namespace shared {

class SignalTracer {
public:
    // Header writer: emits the comma-separated column names for a single
    // trace, prefixed with `trace_name`. (e.g. for NavigationState with
    // trace_name "ekf", might write "ekf_lat,ekf_lon,ekf_alt,ekf_roll,…".)
    using HeaderFn = std::function<void(std::ostream&, std::string_view trace_name)>;

    // Value writer: emits the comma-separated values for one tick.
    // `typed_value` is a `const T*` — cast back at call time. Wrapped at
    // registration via the typed `register_type<T>` overload below.
    using ValueFn  = std::function<void(std::ostream&, const void* typed_value)>;

    // Register a (header, value) pair for type T. The value writer takes
    // a `const T&` directly; the type-erased wrapper does the cast.
    template<typename T>
    void register_type(HeaderFn header_writer,
                       std::function<void(std::ostream&, const T&)> value_writer) {
        const void* tid = detail::type_tag<T>::id();
        if (serializers_.find(tid) != serializers_.end()) {
            detail::invalid_argument(
                "SignalTracer::register_type: serializer already registered "
                "for this type");
        }
        Serializer s;
        s.header_fn = std::move(header_writer);
        s.value_fn  = [vw = std::move(value_writer)](std::ostream& os, const void* p) {
            vw(os, *static_cast<const T*>(p));
        };
        serializers_.emplace(tid, std::move(s));
    }

    // Add a trace from a graph port. `path` is "block.port" — same format
    // as Graph::connect's string-keyed wiring. `trace_name` becomes the
    // column-name prefix in CSV. Throws on missing block, missing port,
    // or no registered serializer for the port's type.
    void add_trace(const Graph&     graph,
                   std::string_view path,
                   std::string_view trace_name) {
        const auto dot = path.find('.');
        if (dot == std::string_view::npos) {
            detail::invalid_argument(
                std::string("SignalTracer::add_trace: path '") +
                std::string(path) + "' missing '.' separator");
        }
        const std::string_view block_name = path.substr(0, dot);
        const std::string_view port_name  = path.substr(dot + 1);

        Block* b = graph.find(block_name);
        if (!b) {
            detail::invalid_argument(
                std::string("SignalTracer::add_trace: block '") +
                std::string(block_name) + "' not in graph");
        }
        IPort* p = b->port(port_name);
        if (!p) {
            detail::invalid_argument(
                std::string("SignalTracer::add_trace: port '") +
                std::string(port_name) + "' not on block '" +
                std::string(block_name) + "'");
        }
        const auto it = serializers_.find(p->type_id());
        if (it == serializers_.end()) {
            detail::invalid_argument(
                std::string("SignalTracer::add_trace: no serializer "
                            "registered for the type of port '") +
                std::string(path) + "'");
        }
        traces_.push_back(Trace{std::string(trace_name), p, it->second});
    }

    // CSV header: "t_s," + each trace's column names, in add order.
    void write_header(std::ostream& out) const {
        out << "t_s";
        for (const auto& t : traces_) {
            out << ",";
            t.serializer.header_fn(out, t.trace_name);
        }
        out << "\n";
    }

    // CSV row: time (in seconds) + each trace's values, in add order.
    void write_row(std::ostream& out, uint64_t time_us) const {
        out << (static_cast<double>(time_us) * 1e-6);
        for (const auto& t : traces_) {
            out << ",";
            t.serializer.value_fn(out, t.port->read_erased());
        }
        out << "\n";
    }

    std::size_t trace_count() const { return traces_.size(); }

private:
    struct Serializer {
        HeaderFn header_fn;
        ValueFn  value_fn;
    };

    struct Trace {
        std::string  trace_name;
        const IPort* port;
        Serializer   serializer;  // copy, not pointer — avoids invalidation
                                  // if the serializer map ever rehashes.
    };

    std::unordered_map<const void*, Serializer> serializers_;
    std::vector<Trace>                          traces_;
};

} // namespace shared
