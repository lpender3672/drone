#pragma once

#include <algorithm>

#include "../core/block.hpp"
#include "../data/state.hpp"
#include "../../../shared/types/state.h"

namespace sim {

/**
 * Altitude-hold outer loop for vertical-takeoff vehicles.
 *
 * Input:  NavigationState  — reads geodetic altitude (position.z) and vertical velocity.
 * Output: Scalar           — thrust command in [hover_thrust - out_clamp, hover_thrust + out_clamp].
 *
 * Control law:
 *   thrust = hover + Kp·(z_ref - z) + Ki·∫err - Kd·v_up
 *
 * Uses the estimator's own vertical velocity instead of a numerical d/dt on
 * altitude — the latter spikes every time a GNSS/baro/mag update bumps the
 * EKF's position estimate, which destabilises the loop.
 */
template<typename NavigationStateT = shared::NavigationState>
class AltitudeHoldBlock : public TypedBlock<NavigationStateT, Scalar> {
public:
    struct Params {
        // Initial tuning for a 0.5 kg quad with the default motor model. Keeps
        // altitude within a few metres of setpoint over a 2-minute hover in sim.
        // Exact tuning will want per-vehicle iteration; the gains interact with
        // the attitude loop (thrust headroom for roll/pitch/yaw torques).
        double kp           = 0.10;
        double ki           = 0.03;
        double kd           = 0.40;
        // Hover baseline. Kept low (below the true hover point ~0.775 for the
        // sim's default quad) so the mixer keeps room for roll/pitch/yaw torque
        // commands — each motor is `thrust ± roll ± pitch ± yaw`, and too-high
        // base thrust saturates the torque channels. Integral term bridges the
        // gap to true hover.
        double hover_thrust = 0.70;
        double out_clamp    = 0.20;   // thrust span around hover, each side
        double int_clamp    = 3.0;    // ≈ integral authority: Ki·int_clamp = 0.09
    };

    AltitudeHoldBlock(const std::string& name, uint32_t update_period_us = 1000)
        : TypedBlock<NavigationStateT, Scalar>(name, "state", "thrust", update_period_us) {}

    void set_params(const Params& p) { params_ = p; }
    const Params& params() const { return params_; }

    void set_setpoint_m(double z_ref) { setpoint_m_ = z_ref; }
    double setpoint_m() const { return setpoint_m_; }

    void reset() {
        integral_     = 0.0;
        last_thrust_  = params_.hover_thrust;
        has_updated_  = false;
    }

    bool update(uint64_t current_time_us) override {
        if (!this->is_due(current_time_us)) return false;
        const double dt_s = this->get_dt_us(current_time_us) * 1e-6;
        this->mark_updated(current_time_us);

        const auto& state = this->input_.get();
        const double error = setpoint_m_ - state.position.z();
        const double v_up  = -state.velocity.z();  // NED z-down → up = -z

        // Conditional integration (anti-windup): compute the would-be thrust,
        // only accumulate the integral when the output isn't saturated, or when
        // integration would move the output back toward the linear range.
        const double p_term = params_.kp * error;
        const double d_term = -params_.kd * v_up;
        const double i_term_prev = params_.ki * integral_;
        const double unsat = params_.hover_thrust + p_term + i_term_prev + d_term;

        const double upper = params_.hover_thrust + params_.out_clamp;
        const double lower = params_.hover_thrust - params_.out_clamp;
        const bool saturated_high = unsat > upper && error > 0.0;  // error would push further up
        const bool saturated_low  = unsat < lower && error < 0.0;  // error would push further down

        if (has_updated_ && !saturated_high && !saturated_low) {
            integral_ = std::clamp(integral_ + error * dt_s,
                                   -params_.int_clamp, params_.int_clamp);
        }
        has_updated_ = true;

        double thrust = std::clamp(params_.hover_thrust
                                   + p_term
                                   + params_.ki * integral_
                                   + d_term,
                                   lower, upper);

        this->output_.value = Scalar(thrust, current_time_us);
        last_thrust_  = thrust;
        return true;
    }

private:
    Params  params_;
    double  setpoint_m_  = 0.0;
    double  integral_    = 0.0;
    double  last_thrust_ = 0.70;
    bool    has_updated_ = false;
};

} // namespace sim
