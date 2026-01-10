#pragma once

#include "../core/block.hpp"
#include "../data/state.hpp"
#include "../data/control_efforts.hpp"

namespace sim {

/**
 * Interface for dynamics model blocks.
 * Takes control efforts, propagates physics, outputs true state.
 * 
 * Template parameter N specifies the number of control channels.
 * Runs at high rate internally (sub-stepping) for numerical stability.
 */
template<int N>
class DynamicsModel : public Block {
public:
    using EffortsType = ControlEfforts<N>;

    DynamicsModel(const std::string& name, double update_rate_hz, double internal_rate_hz = 10000.0)
        : Block(name, update_rate_hz)
        , internal_rate_hz_(internal_rate_hz)
        , internal_dt_(1.0 / internal_rate_hz)
    {}

    // Set control commands (called by controller)
    void set_control_efforts(const EffortsType& efforts) {
        control_efforts_ = efforts;
    }

    // Get current true state
    const State& true_state() const { return state_; }

    // Reset to initial conditions
    virtual void reset(const State& initial_state) {
        state_ = initial_state;
        control_efforts_ = EffortsType{};
    }

    // Block interface
    bool update(double current_time_s) override {
        if (!is_due(current_time_s)) return false;
        
        double dt = update_period_s_;
        if (last_update_time_s_ > 0) {
            dt = current_time_s - last_update_time_s_;
        }
        
        // Sub-step the dynamics for numerical stability
        int num_steps = static_cast<int>(std::ceil(dt / internal_dt_));
        double sub_dt = dt / num_steps;
        
        for (int i = 0; i < num_steps; ++i) {
            step_dynamics(sub_dt);
        }
        
        state_.set_timestamp(current_time_s);
        mark_updated(current_time_s);
        notify_output(state_);
        
        return true;
    }

protected:
    // Override this to implement actual physics
    // Should update state_ based on control_efforts_ and dt
    virtual void step_dynamics(double dt) = 0;

    State state_;
    EffortsType control_efforts_;
    
    double internal_rate_hz_;
    double internal_dt_;
};

} // namespace sim
