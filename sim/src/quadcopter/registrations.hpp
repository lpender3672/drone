#pragma once

#include <memory>
#include <string>

#include "../../../shared/blocks/altitude_hold.hpp"
#include "../../../shared/blocks/attitude_controller.hpp"
#include "../../../shared/blocks/pid.hpp"
#include "../../../shared/blocks/quadrotor_ekf.hpp"
#include "../../../shared/blocks/unit_delay.hpp"
#include "../../../shared/core/block_factory.hpp"
#include "../../../shared/data/gnss_origin.hpp"
#include "../blocks/force_disturbance.hpp"
#include "../blocks/sensors_impl.hpp"
#include "../blocks/signal_generator.hpp"
#include "../sim_ekf_params.h"
#include "../../../shared/types/state.h"
#include "dynamics.hpp"
#include "ekf16d.h"

// Factory registrations for the leaf block types used by the sim's flat
// graph spec. The registrations are scoped to a `BlockFactory` instance —
// no global state — so tests can build a fresh factory.
//
// Each ctor reads its scalar tunings from `BlockParams::get_double(key,
// default)`. Keys not present fall through to the block's compiled-in
// defaults, so a minimal JSON spec like `{"type": "pid", "name": "p"}` is
// valid and produces a default-tuned PID. Vector quantities (3-axis gains,
// inertia, GnssOrigin) are split into _x/_y/_z (or _lat/_lon/_alt) keys.
//
// Tier 2 deliverable: `register_quadrotor_blocks(factory)` is what
// `sim/src/main.cpp` calls before handing the factory to `load_graph_json`.

namespace sim {

namespace detail {

inline shared::PidParams pid_params_from(const shared::BlockParams& p,
                                         const shared::PidParams&    defaults = {}) {
    shared::PidParams out = defaults;
    out.kp           = p.get_double("kp",           out.kp);
    out.ki           = p.get_double("ki",           out.ki);
    out.kd           = p.get_double("kd",           out.kd);
    out.output_min   = p.get_double("output_min",   out.output_min);
    out.output_max   = p.get_double("output_max",   out.output_max);
    out.integral_min = p.get_double("integral_min", out.integral_min);
    out.integral_max = p.get_double("integral_max", out.integral_max);
    return out;
}

inline shared::GnssOrigin origin_from(const shared::BlockParams& p,
                                      const shared::GnssOrigin&  defaults = {}) {
    shared::GnssOrigin o = defaults;
    o.lat_deg = p.get_double("origin_lat_deg", o.lat_deg);
    o.lon_deg = p.get_double("origin_lon_deg", o.lon_deg);
    o.alt_m   = p.get_double("origin_alt_m",   o.alt_m);
    return o;
}

} // namespace detail

inline void register_quadrotor_blocks(shared::BlockFactory& factory) {
    using shared::BlockParams;

    // ---- Scalar PID ---------------------------------------------------
    factory.register_type("pid", [](const std::string& name, const BlockParams& p) {
        auto block = std::make_unique<shared::PidBlock>(
            name,
            static_cast<uint32_t>(p.get_double("update_period_us", 0)));
        block->set_params(detail::pid_params_from(p));
        return block;
    });

    // ---- Altitude hold ------------------------------------------------
    factory.register_type("altitude_hold", [](const std::string& name, const BlockParams& p) {
        auto block = std::make_unique<shared::AltitudeHoldBlock<shared::NavigationState>>(
            name,
            static_cast<uint32_t>(p.get_double("update_period_us", 1000)));
        shared::AltitudeHoldBlock<shared::NavigationState>::Params params;
        params.kp           = p.get_double("kp",           params.kp);
        params.ki           = p.get_double("ki",           params.ki);
        params.kd           = p.get_double("kd",           params.kd);
        params.hover_thrust = p.get_double("hover_thrust", params.hover_thrust);
        params.out_clamp    = p.get_double("out_clamp",    params.out_clamp);
        params.int_clamp    = p.get_double("int_clamp",    params.int_clamp);
        block->set_params(params);
        block->set_setpoint_m(p.get_double("setpoint_m", 0.0));
        return block;
    });

    // ---- Attitude PID controller (cascaded: outer P, inner PID per axis) -
    // Per-axis rate-PID gains are flattened as {axis}_{field}, where axis ∈
    // {roll, pitch, yaw} matches the rate_pid[0..2] index (NED-body). The
    // outer attitude P loop is one gain per axis (kp_attitude_x/y/z) plus
    // a single max_rate_setpoint clamp.
    factory.register_type("attitude_pid", [](const std::string& name, const BlockParams& p) {
        shared::AttitudePidController::Params params;

        params.kp_attitude.x()    = p.get_double("kp_attitude_x",    params.kp_attitude.x());
        params.kp_attitude.y()    = p.get_double("kp_attitude_y",    params.kp_attitude.y());
        params.kp_attitude.z()    = p.get_double("kp_attitude_z",    params.kp_attitude.z());
        params.max_rate_setpoint  = p.get_double("max_rate_setpoint", params.max_rate_setpoint);

        const char* axes[3] = { "roll", "pitch", "yaw" };
        for (int i = 0; i < 3; ++i) {
            const std::string ax = std::string(axes[i]) + "_";
            shared::PidParams& rp = params.rate_pid[i];
            rp.kp           = p.get_double(ax + "kp",           rp.kp);
            rp.ki           = p.get_double(ax + "ki",           rp.ki);
            rp.kd           = p.get_double(ax + "kd",           rp.kd);
            rp.integral_min = p.get_double(ax + "integral_min", rp.integral_min);
            rp.integral_max = p.get_double(ax + "integral_max", rp.integral_max);
            rp.output_min   = p.get_double(ax + "output_min",   rp.output_min);
            rp.output_max   = p.get_double(ax + "output_max",   rp.output_max);
        }

        return std::make_unique<shared::AttitudePidController>(
            name,
            params,
            static_cast<uint32_t>(p.get_double("update_period_us", 1000)));
    });

    // ---- Quadrotor 6-DoF dynamics + 4 motors --------------------------
    factory.register_type("quadrotor_dynamics", [](const std::string& name, const BlockParams& p) {
        sim::quadcopter::QuadrotorDynamics::Params dp;
        dp.mass         = p.get_double("mass",         dp.mass);
        dp.arm_length   = p.get_double("arm_length",   dp.arm_length);
        dp.inertia.x()  = p.get_double("inertia_x",    dp.inertia.x());
        dp.inertia.y()  = p.get_double("inertia_y",    dp.inertia.y());
        dp.inertia.z()  = p.get_double("inertia_z",    dp.inertia.z());
        dp.drag_coeff   = p.get_double("drag_coeff",   dp.drag_coeff);
        dp.gravity      = p.get_double("gravity",      dp.gravity);
        dp.motor.tau         = p.get_double("motor_tau",       dp.motor.tau);
        dp.motor.omega_max   = p.get_double("motor_omega_max", dp.motor.omega_max);
        dp.propeller.k_t     = p.get_double("prop_k_t",        dp.propeller.k_t);
        dp.propeller.k_q     = p.get_double("prop_k_q",        dp.propeller.k_q);
        dp.propeller.d       = p.get_double("prop_d",          dp.propeller.d);
        dp.propeller.rho     = p.get_double("prop_rho",        dp.propeller.rho);
        return std::make_unique<sim::quadcopter::QuadrotorDynamics>(
            name,
            dp,
            static_cast<uint32_t>(p.get_double("update_period_us",   1000)),
            static_cast<uint32_t>(p.get_double("internal_period_us", 10000)));
    });

    // ---- Sim sensors --------------------------------------------------
    // ImuSensor / GpsSensor / BaroSensor / MagSensor are templated on
    // TrueState; sim's TrueState wrapper is the only instantiation we need.
    factory.register_type("imu_sensor", [](const std::string& name, const BlockParams& p) {
        auto block = std::make_unique<sim::ImuSensor<shared::TrueState>>(
            name,
            static_cast<uint32_t>(p.get_double("update_period_us", 1000)),
            static_cast<uint32_t>(p.get_double("latency_us",       0)));
        sim::ImuSensor<shared::TrueState>::NoiseParams np;
        np.gyro_noise_stddev  = p.get_double("gyro_noise_stddev",  np.gyro_noise_stddev);
        np.gyro_bias_stddev   = p.get_double("gyro_bias_stddev",   np.gyro_bias_stddev);
        np.accel_noise_stddev = p.get_double("accel_noise_stddev", np.accel_noise_stddev);
        np.accel_bias_stddev  = p.get_double("accel_bias_stddev",  np.accel_bias_stddev);
        block->set_noise_params(np);
        return block;
    });

    factory.register_type("gps_sensor", [](const std::string& name, const BlockParams& p) {
        auto block = std::make_unique<sim::GpsSensor<shared::TrueState>>(
            name,
            static_cast<uint32_t>(p.get_double("update_period_us", 100000)),
            static_cast<uint32_t>(p.get_double("latency_us",       80000)));
        sim::GpsSensor<shared::TrueState>::NoiseParams np;
        np.position_stddev = p.get_double("position_stddev", np.position_stddev);
        np.altitude_stddev = p.get_double("altitude_stddev", np.altitude_stddev);
        np.velocity_stddev = p.get_double("velocity_stddev", np.velocity_stddev);
        block->set_noise_params(np);
        block->set_origin(detail::origin_from(p));
        return block;
    });

    factory.register_type("baro_sensor", [](const std::string& name, const BlockParams& p) {
        auto block = std::make_unique<sim::BaroSensor<shared::TrueState>>(
            name,
            static_cast<uint32_t>(p.get_double("update_period_us", 50000)),
            static_cast<uint32_t>(p.get_double("latency_us",       20000)));
        sim::BaroSensor<shared::TrueState>::NoiseParams np;
        np.altitude_noise_stddev = p.get_double("altitude_noise_stddev", np.altitude_noise_stddev);
        np.bias_stddev           = p.get_double("bias_stddev",           np.bias_stddev);
        block->set_noise_params(np);
        block->set_ground_alt_m(p.get_double("ground_alt_m",
                                             detail::origin_from(p).alt_m));
        return block;
    });

    factory.register_type("mag_sensor", [](const std::string& name, const BlockParams& p) {
        auto block = std::make_unique<sim::MagSensor<shared::TrueState>>(
            name,
            static_cast<uint32_t>(p.get_double("update_period_us", 20000)),
            static_cast<uint32_t>(p.get_double("latency_us",       0)));
        sim::MagSensor<shared::TrueState>::NoiseParams np;
        np.noise_stddev = p.get_double("noise_stddev", np.noise_stddev);
        block->set_noise_params(np);
        return block;
    });

    // ---- EKF block (defaults: SIM_DATA_PARAMS observer) ---------------
    // The QuadrotorEkfBlock holds an INavObserver via unique_ptr. The
    // factory variants differ in which observer they instantiate plus
    // which measurement types are enabled — that's the recompile-free
    // A/B knob. ekf_compare.json picks `ekf16d_imu_only` to compare
    // dead-reckoning against full-aided.
    auto make_ekf_block = [](const std::string& name, const BlockParams& p,
                             bool gnss_on, bool baro_on, bool mag_on) {
        auto observer = std::make_unique<EKF16d>(SIM_DATA_PARAMS);
        auto block = std::make_unique<shared::QuadrotorEkfBlock>(
            name,
            std::move(observer),
            static_cast<uint32_t>(p.get_double("update_period_us", 1000)));
        block->set_origin(detail::origin_from(p));
        block->set_gnss_enabled(gnss_on);
        block->set_baro_enabled(baro_on);
        block->set_mag_enabled(mag_on);
        return block;
    };

    factory.register_type("ekf16d", [make_ekf_block](const std::string& name, const BlockParams& p) {
        return make_ekf_block(name, p, true, true, true);
    });

    factory.register_type("ekf16d_imu_only", [make_ekf_block](const std::string& name, const BlockParams& p) {
        return make_ekf_block(name, p, false, false, false);
    });

    // ---- Test-harness signal/disturbance blocks -----------------------
    factory.register_type("waveform_generator", [](const std::string& name, const BlockParams& p) {
        sim::WaveformParams wp;
        const std::string type = p.get_string("waveform", "step");
        if      (type == "impulse") wp.type = sim::Waveform::Impulse;
        else if (type == "step")    wp.type = sim::Waveform::Step;
        else if (type == "ramp")    wp.type = sim::Waveform::Ramp;
        else if (type == "sine")    wp.type = sim::Waveform::Sine;
        else shared::detail::invalid_argument(
            "waveform_generator '" + name + "': unknown waveform type '" + type + "'");
        wp.amplitude    = p.get_double("amplitude",    wp.amplitude);
        wp.start_time_s = p.get_double("start_time_s", wp.start_time_s);
        wp.duration_s   = p.get_double("duration_s",   wp.duration_s);
        wp.frequency_hz = p.get_double("frequency_hz", wp.frequency_hz);
        wp.bias         = p.get_double("bias",         wp.bias);
        return std::make_unique<sim::WaveformGenerator>(
            name, wp,
            static_cast<uint32_t>(p.get_double("update_period_us", 1000)));
    });

    factory.register_type("torque_disturbance", [](const std::string& name, const BlockParams& p) {
        const shared::Vec3 axis(p.get_double("axis_x", 1.0),
                                p.get_double("axis_y", 0.0),
                                p.get_double("axis_z", 0.0));
        return std::make_unique<sim::TorqueDisturbanceBlock>(
            name, axis,
            static_cast<uint32_t>(p.get_double("update_period_us", 1000)));
    });

    // ---- Unit-delay (Tier 2 algebraic-loop breaker) -------------------
    // The closed-loop graph (sensors → EKF → controller → dynamics → sensors)
    // is a real cycle. Tier 2 resolves this with a UnitDelayBlock<T> on the
    // back-edge — `is_delay()` makes topo_order ignore the incoming edge,
    // matching the Z-domain z⁻¹ convention. quad.json wires
    // dynamics.state → truth_delay.in → sensor.true_state, which mirrors
    // the original Plant's "sensors see previous-step truth" comment.
    factory.register_type("unit_delay_truestate", [](const std::string& name, const BlockParams& p) {
        return std::make_unique<shared::UnitDelayBlock<shared::TrueState>>(
            name,
            shared::TrueState{},
            static_cast<uint32_t>(p.get_double("update_period_us", 1000)));
    });

    factory.register_type("force_disturbance", [](const std::string& name, const BlockParams& p) {
        const shared::Vec3 dir(p.get_double("dir_x", 0.0),
                               p.get_double("dir_y", 0.0),
                               p.get_double("dir_z", 1.0));
        return std::make_unique<sim::ForceDisturbanceBlock>(
            name, dir,
            static_cast<uint32_t>(p.get_double("update_period_us", 1000)));
    });
}

} // namespace sim
