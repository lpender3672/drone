#include <iostream>
#include <iomanip>
#include <fstream>
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
#include "blocks/signal_generator.hpp"
#include "blocks/force_disturbance.hpp"
#include "blocks/altitude_hold.hpp"

// EKF params and concrete observer
#include "sim_ekf_params.h"
#include "ekf16d.h"

using namespace sim;
using namespace sim::quadcopter;

int main() {
    std::cout << "=== Quadrotor Closed-Loop Impulse Response ===\n\n";

    Simulation sim_runner;  // default 1 ms step

    // =========================================
    // Create blocks (in execution order)
    // =========================================
    auto dist_gen    = sim_runner.add_block(std::make_unique<WaveformGenerator>("dist_gen",    WaveformParams{}, 1000));
    auto dist_torque = sim_runner.add_block(std::make_unique<TorqueDisturbanceBlock>("dist_torque", Vec3(1, 0, 0), 1000));

    auto imu_sensor  = sim_runner.add_block(std::make_unique<ImuSensor<TrueState>>("imu",    1000,      0));
    auto gnss_sensor = sim_runner.add_block(std::make_unique<GpsSensor<TrueState>>("gnss", 100000,  80000));
    auto baro_sensor = sim_runner.add_block(std::make_unique<BaroSensor<TrueState>>("baro",  50000,  20000));
    auto mag_sensor  = sim_runner.add_block(std::make_unique<MagSensor<TrueState>>("mag",   20000,      0));

    auto ekf_block   = sim_runner.add_block(std::make_unique<QuadrotorEkfBlock>(
        "ekf", std::make_unique<EKF16d>(SIM_DATA_PARAMS), 1000));
    auto alt_hold    = sim_runner.add_block(
        std::make_unique<AltitudeHoldBlock<NavigationState>>("alt_hold", 1000));

    auto controller  = sim_runner.add_block(std::make_unique<AttitudePidController>("controller", 1000));

    auto dynamics    = sim_runner.add_block(std::make_unique<QuadrotorDynamics>("dynamics", 1000, 100));

    // =========================================
    // Configure disturbance waveform
    // =========================================
    // 0.1 N·m roll torque impulse for 10 ms at t = 1 s
    // I_x = 0.0035 kg·m²  →  Δω ≈ 0.1/0.0035 × 0.01 ≈ 0.29 rad/s roll kick
    WaveformParams dist_params;
    dist_params.type         = Waveform::Impulse;
    dist_params.amplitude    = 0.1;
    dist_params.start_time_s = 1.0;
    dist_params.duration_s   = 0.01;
    dist_gen->set_params(dist_params);

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

    TrueState init;
    init.position         = Vec3(0.0, 0.0, -1.0);
    init.velocity         = Vec3::Zero();
    init.attitude         = Quat::Identity();
    init.angular_velocity = Vec3::Zero();
    dynamics->reset(init);

    baro_sensor->set_ground_alt_m(ekf_block->origin_alt_m_);

    ekf_block->initialize(init);
    ekf_block->set_gnss_enabled(true);
    ekf_block->set_baro_enabled(true);
    ekf_block->set_mag_enabled(true);

    // =========================================
    // Wire port connections
    // =========================================
    connect(dist_gen->output(),    dist_torque->input());
    connect(dist_torque->output(), dynamics->disturbance_torque_input());

    connect(dynamics->output(), imu_sensor->input());
    connect(dynamics->output(), gnss_sensor->input());
    connect(dynamics->output(), baro_sensor->input());
    connect(dynamics->output(), mag_sensor->input());

    connect(imu_sensor->output(),  ekf_block->imu_input());
    connect(gnss_sensor->output(), ekf_block->gnss_input());
    connect(baro_sensor->output(), ekf_block->baro_input());
    connect(mag_sensor->output(),  ekf_block->mag_input());

    connect(ekf_block->output(),   alt_hold->input());
    connect(alt_hold->output(),    controller->thrust_input());
    connect(ekf_block->output(),   controller->state_input());
    connect(controller->output(),  dynamics->input());

    // =========================================
    // Attitude reference (roll/pitch/yaw only — thrust driven by alt_hold)
    // =========================================
    AttitudeReference ref;
    ref.set_roll(0.0);
    ref.set_pitch(0.0);
    ref.set_yaw(0.0);
    controller->reference_input().set(ref);

    alt_hold->set_setpoint_m(ekf_block->origin_alt_m_ - init.position.z());

    // =========================================
    // CSV output
    // =========================================
    std::ofstream csv("impulse_response.csv");
    csv << "t_s,roll_deg,pitch_deg,yaw_deg,roll_rate_dps,pitch_rate_dps,pos_z_m,disturbance_nm,"
           "ekf_roll_deg,ekf_pitch_deg,ekf_yaw_deg,ekf_pos_z_m\n";
    csv << std::fixed << std::setprecision(6);

    // =========================================
    // Run loop: 2 minutes, 1 ms steps
    // =========================================
    constexpr uint64_t DT_US  = 1000;
    constexpr uint64_t END_US = 120000000;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  t(s)   roll(deg)  pitch(deg)  pos_z(m)  dist(N·m)\n";
    std::cout << "-----------------------------------------------------------\n";

    for (uint64_t t = 0; t < END_US; t += DT_US) {
        sim_runner.step();

        const auto& s      = dynamics->output().get();
        Vec3 euler         = s.euler_angles();
        Vec3 omega_dps     = s.angular_velocity * (180.0 / M_PI);
        double dist_nm     = dist_gen->value();

        const auto& ekf_est = ekf_block->output().get();
        Vec3 ekf_euler      = ekf_est.euler_angles();

        csv << (t / 1e6)
            << "," << (euler.x() * 180.0 / M_PI)
            << "," << (euler.y() * 180.0 / M_PI)
            << "," << (euler.z() * 180.0 / M_PI)
            << "," << omega_dps.x()
            << "," << omega_dps.y()
            << "," << s.position.z()
            << "," << dist_nm
            << "," << (ekf_euler.x() * 180.0 / M_PI)
            << "," << (ekf_euler.y() * 180.0 / M_PI)
            << "," << (ekf_euler.z() * 180.0 / M_PI)
            << "," << ekf_est.position.z()
            << "\n";

        if (t % 1000000 == 0) {
            std::cout << std::setw(6)  << (t / 1e6)
                      << "   roll=" << std::setw(7) << (euler.x() * 180.0 / M_PI)
                      << " pitch=" << std::setw(7) << (euler.y() * 180.0 / M_PI)
                      << " pos_z=" << std::setw(7) << s.position.z()
                      << " dist=" << std::setw(6) << dist_nm
                      << "\n";
        }
    }

    csv.close();
    std::cout << "\nWrote impulse_response.csv\n";

    return 0;
}
