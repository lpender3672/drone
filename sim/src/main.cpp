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
    //auto ahrs = sim.add_block(std::make_unique<AhrsSensor<shared::TrueState>>("ahrs", 100.0, 0.005));
    auto imu = sim.add_block(std::make_unique<ImuSensor<shared::TrueState>>("imu", 1000.0, 0.001));
    auto gps = sim.add_block(std::make_unique<GpsSensor<shared::TrueState>>("gps", 10.0, 0.1));

    // Observer (passthrough for now - uses AHRS directly)
    //auto observer = sim.add_block(std::make_unique<PassthroughObserver>("observer"));

    // =========================================
    // Configure blocks
    // =========================================

    // Dynamics parameters (roughly a small 500g quad)
    QuadrotorDynamics::Params dyn_params;
    dyn_params.mass = 0.5;
    dyn_params.arm_length = 0.175;
    dyn_params.motor.tau = 0.02;
    dyn_params.motor.omega_max = 2500.0;
    dyn_params.propeller.k_t = 1.5e-7;
    dyn_params.propeller.k_q = 2.0e-9;
    dynamics->set_params(dyn_params);

    
    return 0;
}
