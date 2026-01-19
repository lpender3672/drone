#include <iostream>
#include <memory>
#include <cmath>

// Core
#include "core/simulation.hpp"

// Blocks
#include "quadcopter/dynamics.hpp"
#include "quadcopter/controller.hpp"
#include "quadcopter/motor.hpp"

#include "blocks/observer.hpp"
#include "blocks/reference_generator.hpp"
#include "blocks/sensors_impl.hpp"

using namespace sim;
using namespace sim::quadcopter;

int main() {
    std::cout << "=== Quadrotor Simulation Framework (Eigen) ===\n\n";

    // Create simulation with 1ms timestep
    Simulation sim(0.001);
    DataLogger logger;

    // =========================================
    // Create blocks
    // =========================================

    // Reference generator
    auto ref_gen = sim.add_block(std::make_unique<ScalarReferenceGenerator>("ref_gen"));

    // Controller (1kHz)
    auto controller = sim.add_block(std::make_unique<AttitudePidController>("controller", 1000.0));

    // Dynamics model (1kHz update, 10kHz internal)
    auto dynamics = sim.add_block(std::make_unique<QuadrotorDynamics>("dynamics", 1000.0, 10000.0));

    // Sensors
    auto ahrs = sim.add_block(std::make_unique<AhrsSensor<shared::TrueState>>("ahrs", 100.0, 0.005));
    auto imu = sim.add_block(std::make_unique<ImuSensor<shared::TrueState>>("imu", 1000.0, 0.001));
    auto gps = sim.add_block(std::make_unique<GpsSensor<shared::TrueState>>("gps", 10.0, 0.1));

    // Observer (passthrough for now - uses AHRS directly)
    auto observer = sim.add_block(std::make_unique<PassthroughObserver>("observer"));

    // =========================================
    // Configure blocks
    // =========================================

    // Dynamics parameters (roughly a small 500g quad)
    QuadrotorDynamics::Params dyn_params;
    dyn_params.mass = 0.5;
    dyn_params.arm_length = 0.175;
    dyn_params.motor.tau = 0.02;
    dyn_params.motor.omega_max = 2500.0;
    dyn_params.motor.v_supply = 11.1;
    dyn_params.propeller.k_t = 1.5e-7;
    dyn_params.propeller.k_q = 2.0e-9;
    dynamics->set_params(dyn_params);

    // Initial state: slightly off-level, 1m above ground
    State initial_state;
    initial_state.position = Vec3(0.0, 0.0, -1.0);  // NED: -1m is 1m up
    initial_state.set_from_euler(0.05, 0.03, 0.0);  // Small initial tilt
    dynamics->reset(initial_state);
    observer->reset(initial_state);

    // Wire up sensor sources
    ahrs->set_true_state_source(&dynamics->true_state());
    imu->set_true_state_source(&dynamics->true_state());
    gps->set_true_state_source(&dynamics->true_state());

    // Reference: hold level with hover thrust
    Reference hover_ref;
    hover_ref.mode = Reference::Mode::ATTITUDE;
    hover_ref.attitude = Vec3::Zero();
    hover_ref.thrust = 0.5;  // ~hover for our config
    ref_gen->set_constant(hover_ref);

    // After 2 seconds, do a small roll step
    Reference step_ref = hover_ref;
    step_ref.attitude.x() = 0.1;  // 0.1 rad ~ 5.7 degrees roll
    ref_gen->set_step(2.0, hover_ref, step_ref);

    // Attach logger
    sim.attach_logger(&logger);

    // =========================================
    // Main simulation loop
    // =========================================
    
    double sim_duration = 5.0;  // 5 seconds
    double current_time = 0.0;
    double dt = 0.001;  // 1ms
    
    int print_interval_ms = 500;
    int steps_per_print = static_cast<int>(print_interval_ms / (dt * 1000));
    int step_count = 0;

    Vec3 initial_euler = initial_state.euler_angles();
    std::cout << "Running simulation for " << sim_duration << " seconds...\n";
    std::cout << "Initial state: z=" << initial_state.position.z() << "m, roll=" 
              << initial_euler.x() * 180/M_PI << " deg\n\n";

    while (current_time < sim_duration) {
        // Update reference
        ref_gen->update(current_time);

        // Observer produces estimated state
        // Feed sensor data to observer
        auto ahrs_reading = ahrs->get_reading();
        if (ahrs_reading) {
            observer->feed_attitude(*ahrs_reading);
        }
        auto gps_reading = gps->get_reading();
        if (gps_reading) {
            observer->feed_gps(*gps_reading);
        }
        observer->update(current_time);

        // Controller uses estimated state and reference
        controller->set_reference(ref_gen->reference());
        controller->set_estimated_state(observer->estimated_state());
        controller->update(current_time);

        // Feed motor commands to dynamics
        dynamics->set_control_efforts(controller->motor_efforts());
        dynamics->update(current_time);

        // Update sensors (they read from dynamics internally)
        ahrs->update(current_time);
        imu->update(current_time);
        gps->update(current_time);

        // Progress output
        step_count++;
        if (step_count % steps_per_print == 0) {
            const auto& state = dynamics->true_state();
            Vec3 euler = state.euler_angles();
            const auto& ref = ref_gen->reference();
            const auto& motors = controller->motor_efforts();

            std::cout << "t=" << std::fixed << std::setprecision(2) << current_time << "s"
                      << "  z=" << std::setprecision(3) << state.position.z() << "m"
                      << "  roll=" << std::setprecision(1) << euler.x() * 180/M_PI << "°"
                      << " (ref=" << ref.attitude.x() * 180/M_PI << "°)"
                      << "  motors=[" << std::setprecision(2)
                      << motors[0] << "," << motors[1] << "," 
                      << motors[2] << "," << motors[3] << "]\n";
        }

        // Advance time
        current_time += dt;
    }

    std::cout << "\nSimulation complete!\n";

    // Write logged data
    logger.write_csv("sim_");
    std::cout << "Logged " << logger.size() << " data points to CSV files.\n";

    // Final state
    const auto& final_state = dynamics->true_state();
    Vec3 final_euler = final_state.euler_angles();
    std::cout << "\nFinal state:\n"
              << "  Position: [" << final_state.position.transpose() << "] m\n"
              << "  Velocity: [" << final_state.velocity.transpose() << "] m/s\n"
              << "  Attitude: [" << final_euler.x()*180/M_PI << ", " 
              << final_euler.y()*180/M_PI << ", " << final_euler.z()*180/M_PI << "] deg\n";

    return 0;
}
