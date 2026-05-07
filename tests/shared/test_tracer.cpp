#include <gtest/gtest.h>
#include <sstream>

#include <core/graph.hpp>
#include <core/graph_loader.hpp>
#include <core/tracer.hpp>

#include "../mocks/mock_block.h"

using shared::Graph;
using shared::SignalTracer;
using mocks::MockBlock;

namespace {

// Trivial scalar-double serializer used by most tests below. Header writes
// the trace name verbatim; value writes the number.
void register_double(SignalTracer& t) {
    t.register_type<double>(
        [](std::ostream& os, std::string_view name) { os << name; },
        [](std::ostream& os, const double& v)       { os << v; });
}

} // namespace

TEST(SignalTracer, EmptyTracerWritesOnlyTimeColumn) {
    SignalTracer t;
    std::ostringstream hdr, row;
    t.write_header(hdr);
    t.write_row(row, /*time_us*/ 0);
    EXPECT_EQ(hdr.str(), "t_s\n");
    EXPECT_EQ(row.str(), "0\n");
}

TEST(SignalTracer, AddTraceMissingBlockThrows) {
    SignalTracer t;
    register_double(t);
    Graph g;
    EXPECT_THROW(t.add_trace(g, "nope.out", "x"), std::invalid_argument);
}

TEST(SignalTracer, AddTraceMissingPortThrows) {
    SignalTracer t;
    register_double(t);
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));
    EXPECT_THROW(t.add_trace(g, "a.nope", "x"), std::invalid_argument);
}

TEST(SignalTracer, AddTracePathMissingDotThrows) {
    SignalTracer t;
    register_double(t);
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));
    EXPECT_THROW(t.add_trace(g, "a_out", "x"), std::invalid_argument);
}

TEST(SignalTracer, AddTraceUnregisteredTypeThrows) {
    // No serializer registered → must throw at add time, not silently
    // produce an unloggable trace.
    SignalTracer t;
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));
    EXPECT_THROW(t.add_trace(g, "a.out", "x"), std::invalid_argument);
}

TEST(SignalTracer, DuplicateTypeRegistrationThrows) {
    SignalTracer t;
    register_double(t);
    EXPECT_THROW(register_double(t), std::invalid_argument);
}

TEST(SignalTracer, ReadsThroughInputPortToWiredOutput) {
    // The tracer reads ports via IPort::read_erased(). For an input port
    // wired to an upstream output, read_erased must follow the source
    // pointer — otherwise traced inputs would always show stale values.
    SignalTracer t;
    register_double(t);

    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    g.connect(*a, a->out(), *b, b->in());

    a->out().set(7.5);

    t.add_trace(g, "a.out", "src");  // output port
    t.add_trace(g, "b.in",  "dst");  // input port wired through to a.out

    std::ostringstream row;
    t.write_row(row, /*time_us*/ 1'000'000);
    EXPECT_EQ(row.str(), "1,7.5,7.5\n");
}

TEST(SignalTracer, WritesHeaderAndRowInTraceOrder) {
    SignalTracer t;
    register_double(t);
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    a->out().set(1.0);
    b->out().set(2.0);

    t.add_trace(g, "b.out", "second");
    t.add_trace(g, "a.out", "first");

    std::ostringstream hdr, row;
    t.write_header(hdr);
    t.write_row(row, /*time_us*/ 250'000);
    EXPECT_EQ(hdr.str(), "t_s,second,first\n");
    EXPECT_EQ(row.str(), "0.25,2,1\n");  // 250 ms → 0.25 s
}

TEST(SignalTracer, MultiColumnSerializerExpandsCleanly) {
    // Real-world types fan out into N columns; verify the column layout
    // round-trips through the comma-separating logic without leading or
    // trailing commas, and that the header/value writers are called with
    // the right trace_name prefix.
    struct Pair { double a; double b; };

    SignalTracer t;
    t.register_type<Pair>(
        [](std::ostream& os, std::string_view n) {
            os << n << "_a," << n << "_b";
        },
        [](std::ostream& os, const Pair& p) {
            os << p.a << "," << p.b;
        });

    Graph g;
    class PairBlock : public shared::Block {
    public:
        explicit PairBlock(const std::string& name) : Block(name, 0), out_("out") {
            register_port(out_);
        }
        bool update(uint64_t) override { return true; }
        shared::OutputPort<Pair>& out() { return out_; }
    private:
        shared::OutputPort<Pair> out_;
    };
    auto* p = g.add_block(std::make_unique<PairBlock>("p"));
    p->out().value = Pair{3.0, 4.0};

    t.add_trace(g, "p.out", "pp");

    std::ostringstream hdr, row;
    t.write_header(hdr);
    t.write_row(row, /*time_us*/ 0);
    EXPECT_EQ(hdr.str(), "t_s,pp_a,pp_b\n");
    EXPECT_EQ(row.str(), "0,3,4\n");
}

// ---- load_traces_json -----------------------------------------------

TEST(LoadTracesJson, EmptyDocAddsNoTraces) {
    SignalTracer t;
    register_double(t);
    Graph g;
    shared::load_traces_json("{}", g, t);
    EXPECT_EQ(t.trace_count(), 0u);
}

TEST(LoadTracesJson, DocWithoutTracesKeyIsNoOp) {
    SignalTracer t;
    register_double(t);
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));
    shared::load_traces_json(R"({"blocks": []})", g, t);
    EXPECT_EQ(t.trace_count(), 0u);
}

TEST(LoadTracesJson, PopulatesTracerInOrder) {
    SignalTracer t;
    register_double(t);
    Graph g;
    auto* a = g.add_block(std::make_unique<MockBlock<>>("a"));
    auto* b = g.add_block(std::make_unique<MockBlock<>>("b"));
    a->out().set(11.0);
    b->out().set(22.0);

    shared::load_traces_json(R"({
        "traces": [
            { "port": "a.out", "name": "first" },
            { "port": "b.out", "name": "second" }
        ]
    })", g, t);

    ASSERT_EQ(t.trace_count(), 2u);
    std::ostringstream row;
    t.write_row(row, 0);
    EXPECT_EQ(row.str(), "0,11,22\n");
}

TEST(LoadTracesJson, MalformedThrows) {
    SignalTracer t;
    register_double(t);
    Graph g;
    g.add_block(std::make_unique<MockBlock<>>("a"));

    // Missing 'name'.
    EXPECT_THROW(
        shared::load_traces_json(R"({"traces":[{"port":"a.out"}]})", g, t),
        std::invalid_argument);

    // 'traces' is not an array.
    EXPECT_THROW(
        shared::load_traces_json(R"({"traces":"oops"})", g, t),
        std::invalid_argument);

    // Reference to a non-existent block (graph.find returns null).
    EXPECT_THROW(
        shared::load_traces_json(R"({
            "traces": [{ "port": "nope.out", "name": "x" }]
        })", g, t),
        std::invalid_argument);
}
