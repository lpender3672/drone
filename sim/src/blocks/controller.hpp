#pragma once

#include "../core/block.hpp"
#include "../data/state.hpp"
#include "../data/reference.hpp"
#include "../data/control_efforts.hpp"

namespace sim {

/**
 * Interface for controller blocks.
 * Takes estimated state and reference, outputs control efforts.
 * 
 * Template parameter N specifies the number of control channels.
 * Implementations: PID, LQR, MPC, etc.
 */
template<int N>
class Controller : public Block {
public:
    using EffortsType = ControlEfforts<N>;

    Controller(const std::string& name, double update_rate_hz)
        : Block(name, update_rate_hz)
    {}

    // Set inputs
    void set_estimated_state(const State& state) { estimated_state_ = state; }
    void set_reference(const Reference& ref) { reference_ = ref; }

    // Get output
    const EffortsType& control_efforts() const { return control_efforts_; }

    // Reset controller state (integrators, etc.)
    virtual void reset() {
        control_efforts_ = EffortsType{};
    }

protected:
    State estimated_state_;
    Reference reference_;
    EffortsType control_efforts_;
};

} // namespace sim
