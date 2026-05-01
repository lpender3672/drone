#pragma once

#include "../core/block.hpp"

namespace shared {

/**
 * Unit-delay block (z⁻¹). Output at tick N equals input at tick N-1.
 *
 * Used by graph-driven composition (Tier 2) to break algebraic loops:
 * topo_order ignores incoming edges to delay blocks (`is_delay() == true`),
 * so a cycle that passes through a UnitDelayBlock is acceptable. The
 * delay introduces a single discrete-time step on the back-edge, which
 * is the standard Simulink/Z-domain way to resolve same-tick feedback.
 *
 * On the very first tick (no prior input), output is the constructor-
 * supplied initial value (defaults to T{}).
 */
template<typename T>
class UnitDelayBlock : public TypedBlock<T, T> {
public:
    explicit UnitDelayBlock(const std::string& name,
                            T initial = T{},
                            uint32_t update_period_us = 0u)
        : TypedBlock<T, T>(name, "in", "out", update_period_us)
        , held_(initial)
    {
        // Make initial value visible at tick 0 *before* the first update.
        this->output_.value = initial;
    }

    bool is_delay() const override { return true; }

    bool update(uint64_t t) override {
        if (!this->is_due(t)) return false;

        // Publish what we latched on the previous tick (or the initial
        // value, if this is the first tick), then capture this tick's
        // input for the next tick.
        this->output_.value = held_;
        held_               = this->input_.get();

        this->mark_updated(t);
        return true;
    }

private:
    T held_;
};

} // namespace shared
