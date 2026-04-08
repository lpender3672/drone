#include <iostream>
#include <iomanip>
#include <memory>
#include <cmath>

// Core
#include "core/simulation.hpp"

// Blocks
#include "quadcopter/dynamics.hpp"
#include "quadcopter/controller.hpp"
#include "quadcopter/motor.hpp"

using namespace sim;
using namespace sim::quadcopter;

int main() {
    std::cout << "=== Quadrotor Simulation ===\n\n";

    Simulation sim_runner(0.001);

    // =========================================
    // Create blocks (sim_runner takes ownership)
    // =========================================
    auto dynamics   = sim_runner.add_block(std::make_unique<QuadrotorDynamics>("dynamics", 1000, 100));
    auto controller = sim_runner.add_block(std::make_unique<AttitudePidController>("controller", 1000));

    // =========================================
    // Configure dynamics
    // =========================================
    // k_t / k_q tuned so hover occurs at ~70% throttle:
    //   T = k_t * rho * omega^2 * d^4,  d=1 absorbs diameter
    //   T_hover = mass*g/4 = 1.226 N
    //   at throttle=0.7: omega = 0.7*2500 = 1750 rad/s
    //   k_t = 1.226 / (1.225 * 1750^2) = 3.27e-7
    QuadrotorDynamics::Params dyn_params;
    dyn_params.mass        = 0.5;
    dyn_params.arm_length  = 0.175;
    dyn_params.inertia     = Vec3(0.0035, 0.0035, 0.0055);
    dyn_params.motor.tau       = 0.02;
    dyn_params.motor.omega_max = 2500.0;
    dyn_params.propeller.k_t   = 3.27e-7;
    dyn_params.propeller.k_q   = 4.25e-9;
    dyn_params.propeller.d     = 1.0;   // d=1 so formula reduces to k_t*rho*omega^2
    dyn_params.propeller.rho   = 1.225;
    dynamics->set_params(dyn_params);

    // Initial state: 1m above ground in NED (z negative = up)
    TrueState init;
    init.position        = Vec3(0.0, 0.0, -1.0);
    init.velocity        = Vec3::Zero();
    init.attitude        = Quat::Identity();
    init.angular_velocity = Vec3::Zero();
    dynamics->reset(init);

    // =========================================
    // Hover reference
    // =========================================
    AttitudeReference ref;
    ref.set_roll(0.0);
    ref.set_pitch(0.0);
    ref.set_yaw(0.0);
    ref.set_thrust(0.70);

    // =========================================
    // Run loop (manual, 1ms steps)
    // =========================================
    constexpr uint64_t DT_US  = 1000;
    constexpr uint64_t END_US = 5000000;  // 5 seconds

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  t(s)   pos_z(m)   vel_z(m/s)   roll(deg)   pitch(deg)\n";
    std::cout << "--------------------------------------------------------------\n";

    for (uint64_t t = 0; t < END_US; t += DT_US) {

        // Perfect observer: wrap true state as observed state for controller
        const TrueState& true_state = dynamics->output().get();
        shared::ObservedState obs(true_state);
        controller->state_input().set(obs);
        controller->reference_input().set(ref);

        // Update controller, then feed efforts into dynamics
        controller->update(t);
        dynamics->input().set(controller->output().get());
        dynamics->update(t);

        // Print at 1 Hz
        if (t % 1000000 == 0) {
            const auto& s = dynamics->output().get();
            Vec3 euler = s.euler_angles();
            std::cout << std::setw(6) << (t / 1e6)
                      << "   " << std::setw(8) << s.position.z()
                      << "   " << std::setw(10) << s.velocity.z()
                      << "   " << std::setw(9) << (euler.x() * 180.0 / M_PI)
                      << "   " << std::setw(10) << (euler.y() * 180.0 / M_PI)
                      << "\n";
        }
    }

    return 0;
}
