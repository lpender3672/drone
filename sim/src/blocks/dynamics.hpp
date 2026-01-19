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
    using InputType = ControlT;
    using OutputType = StateT;
    using Base = TypedBlock<InputType, OutputType>;

    DynamicsBlock(const std::string& name, double update_rate_hz, double internal_rate_hz = 10000.0)
        : Base(name, "efforts", "state", update_rate_hz)
        , internal_rate_hz_(internal_rate_hz)
        , internal_dt_(1.0 / internal_rate_hz)
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
    bool update(double current_time_s) override {
        if (!this->is_due(current_time_s)) return false;
        
        double dt = this->update_period_s_;
        if (this->last_update_time_s_ > 0) {
            dt = current_time_s - this->last_update_time_s_;
        }
        
        // Sub-step the dynamics for numerical stability
        int num_steps = static_cast<int>(std::ceil(dt / internal_dt_));
        double sub_dt = dt / num_steps;
        
        for (int i = 0; i < num_steps; ++i) {
            step_dynamics(sub_dt);
        }
        
        this->output_.value.set_timestamp(current_time_s);
        this->mark_updated(current_time_s);
        this->notify_output(this->output_.value);
        
        return true;
    }

protected:
    // Override this to implement actual physics
    // Should update output_.value based on input_.value and dt
    virtual void step_dynamics(double dt) = 0;

    double internal_rate_hz_;
    double internal_dt_;
};

} // namespace sim
