#include <gtest/gtest.h>
#include "core/block.hpp"
#include "core/simulation.hpp"

using namespace sim;

// ============================================================================
// Minimal concrete block for testing
// ============================================================================

struct Signal {
    double value = 0.0;
    uint64_t timestamp_us = 0;
    // IInterBlockData stub - not needed for port tests
};

// Counts how many times update() is called.
class CounterBlock : public SourceBlock<Signal> {
public:
    int count = 0;

    explicit CounterBlock(const std::string& name, uint32_t period_us = 0)
        : SourceBlock<Signal>(name, "out", period_us) {}

    bool update(uint64_t t) override {
        if (!is_due(t)) return false;
        ++count;
        output_.value.value = static_cast<double>(count);
        output_.value.timestamp_us = t;
        mark_updated(t);
        return true;
    }
};

// Reads its input via get() and records the last value seen.
class RecorderBlock : public TypedBlock<Signal, Signal> {
public:
    double last_seen = -1.0;

    explicit RecorderBlock(const std::string& name, uint32_t period_us = 0)
        : TypedBlock<Signal, Signal>(name, "in", "out", period_us) {}

    bool update(uint64_t t) override {
        if (!is_due(t)) return false;
        last_seen = input_.get().value;
        output_.value = input_.get();
        mark_updated(t);
        return true;
    }
};

// ============================================================================
// InputPort / connect() tests
// ============================================================================

TEST(InputPort, DefaultGetReturnsValueType) {
    InputPort<Signal> p("p");
    EXPECT_DOUBLE_EQ(p.get().value, 0.0);
    EXPECT_FALSE(p.connected);
}

TEST(InputPort, SetUpdatesValueAndConnected) {
    InputPort<Signal> p("p");
    Signal s; s.value = 42.0;
    p.set(s);
    EXPECT_DOUBLE_EQ(p.get().value, 42.0);
    EXPECT_TRUE(p.connected);
}

TEST(InputPort, ConnectReadsThroughSourcePointer) {
    OutputPort<Signal> out("out");
    InputPort<Signal>  in("in");

    Signal s; s.value = 7.0;
    out.set(s);

    connect(out, in);
    EXPECT_TRUE(in.connected);

    // Reading through pointer — sees source value
    EXPECT_DOUBLE_EQ(in.get().value, 7.0);

    // Source updates are immediately visible without re-calling set()
    out.value.value = 99.0;
    EXPECT_DOUBLE_EQ(in.get().value, 99.0);
}

TEST(InputPort, SetAfterConnectUsesLocalValue) {
    // set() does NOT redirect through source; if source is set,
    // get() reads source not local value. This test documents that
    // connect() takes precedence over previous set() calls.
    OutputPort<Signal> out("out");
    InputPort<Signal>  in("in");

    Signal a; a.value = 1.0;
    in.set(a);
    EXPECT_DOUBLE_EQ(in.get().value, 1.0);  // local value before connect

    Signal b; b.value = 2.0;
    out.set(b);
    connect(out, in);

    // After connect, get() reads source, not previous local set
    EXPECT_DOUBLE_EQ(in.get().value, 2.0);
}

// ============================================================================
// CompositeBlock tests
// ============================================================================

class TestComposite : public CompositeBlock {
public:
    CounterBlock*   src;
    RecorderBlock*  rec;

    explicit TestComposite(const std::string& name)
        : CompositeBlock(name)
    {
        src = add_child(std::make_unique<CounterBlock>("src", 0));
        rec = add_child(std::make_unique<RecorderBlock>("rec", 0));
        connect(src->output(), rec->input());
    }
};

TEST(CompositeBlock, UpdatePropagatesInOrder) {
    TestComposite comp("comp");

    comp.update(0);
    EXPECT_EQ(comp.src->count, 1);
    EXPECT_DOUBLE_EQ(comp.rec->last_seen, 1.0);

    comp.update(1);
    EXPECT_EQ(comp.src->count, 2);
    EXPECT_DOUBLE_EQ(comp.rec->last_seen, 2.0);
}

TEST(CompositeBlock, ChildrenSeeConnectedValues) {
    // Verify the recorder sees the source's output via pointer, not a stale copy
    TestComposite comp("comp");

    for (int i = 1; i <= 5; ++i) {
        comp.update(static_cast<uint64_t>(i));
        EXPECT_EQ(comp.rec->last_seen, static_cast<double>(i));
    }
}

// ============================================================================
// Simulation::step() scheduling tests
// ============================================================================

TEST(Simulation, StepRespectsPeriod) {
    Simulation sim(1000);  // 1 ms step

    // 5 ms period block — should fire at t=0,5000,10000,...
    auto* blk = sim.add_block(std::make_unique<CounterBlock>("blk", 5000));

    // Step 10 ms (10 steps of 1 ms each)
    sim.run(0.010);

    // Should have fired at t=0 and t=5000 → 2 updates
    EXPECT_EQ(blk->count, 2);
}

TEST(Simulation, AlwaysDueBlockUpdatesEveryStep) {
    Simulation sim(1000);

    auto* blk = sim.add_block(std::make_unique<CounterBlock>("blk", 0));  // period=0 → always due

    sim.run(0.005);  // 5 steps

    EXPECT_EQ(blk->count, 5);
}

TEST(Simulation, ConnectedBlocksSeeCurrentValues) {
    // Source runs at 2 ms, recorder at 1 ms.
    // Recorder should see source values that advance every 2 steps.
    Simulation sim(1000);

    auto* src = sim.add_block(std::make_unique<CounterBlock>("src", 2000));
    auto* rec = sim.add_block(std::make_unique<RecorderBlock>("rec", 1000));
    connect(src->output(), rec->input());

    // t=0: src fires (count=1), rec fires → sees 1
    sim.step();
    EXPECT_EQ(src->count, 1);
    EXPECT_DOUBLE_EQ(rec->last_seen, 1.0);

    // t=1000: src not due, rec fires → still sees 1
    sim.step();
    EXPECT_EQ(src->count, 1);
    EXPECT_DOUBLE_EQ(rec->last_seen, 1.0);

    // t=2000: src fires (count=2), rec fires → sees 2
    sim.step();
    EXPECT_EQ(src->count, 2);
    EXPECT_DOUBLE_EQ(rec->last_seen, 2.0);
}
