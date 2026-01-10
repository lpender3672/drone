#pragma once

#include "../core/block.hpp"
#include "../data/state.hpp"
#include "../data/reference.hpp"
#include "../data/control_efforts.hpp"
#include "../../../shared/controller.h"

namespace sim {

/**
 * Base class for controller blocks in simulation.
 * 
 * Inherits from:
 *   - Block: for sim scheduling and output callbacks
 *   - shared::ControllerBase<State, Reference, ControlEfforts<N>>: for unified interface + storage
 * 
 * Template parameter N specifies the number of control channels.
 * Implementations: PID, LQR, MPC, etc.
 * 
 * Note: Sim controllers use Block::update(current_time_s) for scheduling.
 * The IController::compute(dt) method provides an alternative entry point.
 */
template<int N>
class Controller : public Block, 
                   public shared::ControllerBase<State, Reference, ControlEfforts<N>> {
public:
    using Base = shared::ControllerBase<State, Reference, ControlEfforts<N>>;
    using EffortsType = ControlEfforts<N>;

    Controller(const std::string& name, double update_rate_hz)
        : Block(name, update_rate_hz)
    {}

    // Default compute() - sim controllers override update() instead
    void compute(double dt) override {
        (void)dt;
        // Sim controllers implement update(current_time_s) which handles timing
        // For direct IController use, derived classes should override this
    }

    // === Sim-specific accessors (aliases for compatibility) ===
    
    void set_estimated_state(const State& state) { Base::set_state(state); }
    const EffortsType& control_efforts() const { return Base::output(); }

protected:
    // Aliases for backward compatibility with existing sim controllers
    // These reference the base class storage
    State& estimated_state_ = Base::state_;
    Reference& reference_ = Base::reference_;
    ControlEfforts<N>& control_efforts_ = Base::control_output_;
};

} // namespace sim
