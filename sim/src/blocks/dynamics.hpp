#pragma once

#include "../core/block.hpp"
#include "../data/state.hpp"

namespace sim {

/**
 * Dynamics model block.
 * Takes control input, propagates physics, outputs state.
 * 
 * Template parameters:
 *   - ControlT: type of control input
 *   - StateT: type of state output
 * Runs at high rate internally (sub-stepping) for numerical stability.
 */
template<typename ControlT, typename StateT>
class DynamicsBlock : public TypedBlock<ControlT, StateT> {
public:

    DynamicsBlock(const std::string& name, uint32_t update_period_us, uint32_t internal_period_us = 10)
        : TypedBlock<ControlT, StateT>(name, "efforts", "state", update_period_us)
        , internal_period_us_(internal_period_us)
    {
        //this->output_.value = State();
    }

    // Get current true state
    const StateT& true_state() const { return this->output_.value; }

    // Reset to initial conditions
    virtual void reset(const StateT& initial_state) {
        this->output_.value = initial_state;
    }

    // Block interface
    bool update(uint64_t current_time_us) override {
        if (!this->is_due(current_time_us)) return false;
        
        uint64_t dt_us = this->last_update_time_us_ > 0 
            ? (current_time_us - this->last_update_time_us_) 
            : static_cast<uint64_t>(this->update_period_us_);
        
        // Sub-step the dynamics for numerical stability
        uint64_t num_steps = (dt_us + internal_period_us_ - 1) / internal_period_us_;
        double sub_dt_s = static_cast<double>(dt_us) / num_steps / 1000000.0;
        
        for (uint64_t i = 0; i < num_steps; ++i) {
            step_dynamics(sub_dt_s);
        }
        
        this->output_.value.set_timestamp(current_time_us);
        this->mark_updated(current_time_us);
        this->notify_output(this->output_.value);
        
        return true;
    }

protected:
    // Override this to implement actual physics
    // Should update output_.value based on input_.value and dt (in seconds)
    virtual void step_dynamics(double dt_s) = 0;

    uint32_t internal_period_us_;
};

} // namespace sim
