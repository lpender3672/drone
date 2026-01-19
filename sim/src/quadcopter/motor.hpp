#pragma once

#include "../core/block.hpp"
#include "../data/state.hpp"
#include <algorithm>
#include <cmath>

namespace sim {

struct PropellerParams {
    double k_t = 0.0;   // Thrust coefficient
    double k_q = 0.0;   // Torque coefficient
    double d = 0.0;     // Diameter (m)
    double rho = 1.225; // Air density (kg/m³)
};


class Motor : public TypedBlock<Scalar, MotorOutput> {
public:
    explicit Motor(const std::string& name,
                   const PropellerParams& prop,
                   double update_rate_hz = 0.0)
        : TypedBlock(name, "throttle", "output", update_rate_hz)
        , prop_(prop) {}

    virtual ~Motor() = default;



protected:
    void compute_aero_outputs() {
        double omega_sq = output_.value.omega() * output_.value.omega();
        output_.value.set_thrust(prop_.k_t * prop_.rho * omega_sq * std::pow(prop_.d, 4));
        output_.value.set_torque(prop_.k_q * prop_.rho * omega_sq * std::pow(prop_.d, 5));
    }

    PropellerParams prop_;
};

class LinearFirstOrderMotor : public Motor {
public:
    struct Params {
        double tau = 0.02;        // Time constant (s)
        double omega_max = 2500.0; // Max angular velocity (rad/s)
    };

    explicit LinearFirstOrderMotor(const std::string& name,
                                   const Params& params,
                                   const PropellerParams& prop,
                                   double update_rate_hz = 0.0)
        : Motor(name, prop, update_rate_hz)
        , params_(params) {}

    bool update(double current_time_s) override {
        if (!is_due(current_time_s)) return false;

        double dt = update_period_s_;
        if (last_update_time_s_ > -1e8) {
            dt = current_time_s - last_update_time_s_;
        }

        mark_updated(current_time_s);

        double throttle = std::clamp(input_.value.value(), 0.0, 1.0);
        double target_omega = throttle * params_.omega_max;

        double alpha = std::exp(-dt / params_.tau);
        output_.value.set_omega(alpha * output_.value.omega() + (1.0 - alpha) * target_omega);
        
        compute_aero_outputs();
        return true;
    }

private:
    Params params_;
};

class NonlinearFirstOrderMotor : public Motor {
public:
    struct Params {
        double J = 1e-5;      // Rotor + propeller inertia (kg·m²)
        double k_m = 0.01;    // Motor torque constant (N·m/A)
        double k_e = 0.01;    // Back-EMF constant (V·s/rad)
        double R = 0.1;       // Winding resistance (Ω)
        double v_supply = 11.1; // Supply voltage (V)
    };

    explicit NonlinearFirstOrderMotor(const std::string& name,
                                      const Params& params,
                                      const PropellerParams& prop,
                                      double update_rate_hz = 0.0)
        : Motor(name, prop, update_rate_hz)
        , params_(params) {}

    bool update(double current_time_s) override {
        if (!is_due(current_time_s)) return false;

        double dt = update_period_s_;
        if (last_update_time_s_ > -1e8) {
            dt = current_time_s - last_update_time_s_;
        }

        mark_updated(current_time_s);

        double throttle = std::clamp(input_.value.value(), 0.0, 1.0);
        double v = throttle * params_.v_supply;

        double omega = output_.value.omega();
        double k_q_eff = prop_.k_q * prop_.rho * std::pow(prop_.d, 5);
        
        double motor_torque = (params_.k_m / params_.R) * (v - params_.k_e * omega);
        double aero_torque = k_q_eff * omega * omega;
        double omega_dot = (motor_torque - aero_torque) / params_.J;

        output_.value.set_omega(std::max(0.0, omega + dt * omega_dot));
        
        compute_aero_outputs();
        return true;
    }

private:
    Params params_;
};

} // namespace sim