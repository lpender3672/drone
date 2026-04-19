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
#include "quadcopter/observer.hpp"
#include "blocks/sensors_impl.hpp"

// EKF params
#include "tuned_ekf_params.h"

using namespace sim;
using namespace sim::quadcopter;

int main() {
    std::cout << "=== Quadrotor Simulation ===\n\n";

    Simulation sim_runner(0.001);

    // =========================================
    // Create blocks
    // =========================================
    auto dynamics    = sim_runner.add_block(std::make_unique<QuadrotorDynamics>("dynamics", 1000, 100));
    auto controller  = sim_runner.add_block(std::make_unique<AttitudePidController>("controller", 1000));
    auto imu_sensor  = sim_runner.add_block(std::make_unique<ImuSensor<TrueState>>("imu", 1000, 1000));
    auto gnss_sensor = sim_runner.add_block(std::make_unique<GpsSensor<TrueState>>("gnss", 100000, 100000));
    auto ekf_block   = sim_runner.add_block(std::make_unique<QuadrotorEkfBlock>("ekf", TEENSY_PTYPE_DATA_PARAMS, 1000));

    // =========================================
    // Configure dynamics
    // =========================================
    QuadrotorDynamics::Params dyn_params;
    dyn_params.mass        = 0.5;
    dyn_params.arm_length  = 0.175;
    dyn_params.inertia     = Vec3(0.0035, 0.0035, 0.0055);
    dyn_params.motor.tau       = 0.02;
    dyn_params.motor.omega_max = 2500.0;
    dyn_params.propeller.k_t   = 3.27e-7;
    dyn_params.propeller.k_q   = 4.25e-9;
    dyn_params.propeller.d     = 1.0;
    dyn_params.propeller.rho   = 1.225;
    dynamics->set_params(dyn_params);

    // Initial state: 1m above ground in NED (z negative = up)
    TrueState init;
    init.position         = Vec3(0.0, 0.0, -1.0);
    init.velocity         = Vec3::Zero();
    init.attitude         = Quat::Identity();
    init.angular_velocity = Vec3::Zero();
    dynamics->reset(init);

    // Initialise EKF from same state
    ekf_block->initialize(init);

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
    std::cout << "  t(s)   pos_z(m)  est_z(m)   vel_z(m/s)   roll(deg)   pitch(deg)\n";
    std::cout << "------------------------------------------------------------------------\n";

    for (uint64_t t = 0; t < END_US; t += DT_US) {

        // Feed true state to sensors
        const TrueState& true_state = dynamics->output().get();
        imu_sensor->input().set(true_state);
        gnss_sensor->input().set(true_state);

        // Update sensors
        imu_sensor->update(t);
        gnss_sensor->update(t);

        // Feed sensor outputs to EKF and update
        ekf_block->imu_input().set(imu_sensor->output().get());
        ekf_block->gnss_input().set(gnss_sensor->output().get());
        ekf_block->update(t);

        // Feed EKF estimate to controller
        controller->state_input().set(ekf_block->output().get());
        controller->reference_input().set(ref);
        controller->update(t);

        // Feed motor efforts to dynamics
        dynamics->input().set(controller->output().get());
        dynamics->update(t);

        // Print at 1 Hz
        if (t % 1000000 == 0) {
            const auto& s   = dynamics->output().get();
            const auto& est = ekf_block->output().get();
            Vec3 euler = s.euler_angles();
            std::cout << std::setw(6)  << (t / 1e6)
                      << "   " << std::setw(7) << s.position.z()
                      << "   " << std::setw(7) << est.position.z()
                      << "   " << std::setw(10) << s.velocity.z()
                      << "   " << std::setw(9) << (euler.x() * 180.0 / M_PI)
                      << "   " << std::setw(10) << (euler.y() * 180.0 / M_PI)
                      << "\n";
        }
    }

    return 0;
}
