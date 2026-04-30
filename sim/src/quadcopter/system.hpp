#pragma once

#include <memory>

#include "../core/block.hpp"
#include "../blocks/sensors_impl.hpp"
#include "../blocks/altitude_hold.hpp"
#include "../data/gnss_origin.hpp"
#include "dynamics.hpp"
#include "controller.hpp"
#include "observer.hpp"
#include "state.hpp"
#include "../../../shared/observer.h"

namespace sim {
namespace quadcopter {

/**
 * Quadrotor vehicle as a CompositeBlock.
 *
 * Owns all vehicle-internal blocks (sensors, EKF, altitude hold, attitude
 * controller, dynamics) and wires them in the constructor. The sim harness
 * only needs to:
 *   1. Inject sensors and a nav observer at construction.
 *   2. Call set_params() on dynamics.
 *   3. Call initialize() with the initial TrueState.
 *   4. Optionally set EKF sensor enables and controller reference.
 *
 * Disturbance inputs are forwarded so test harnesses can inject forces/torques
 * without knowing the internal structure.
 *
 * Template parameters default to the standard simulation sensor types so that
 * `QuadrotorSystem` (the alias below) needs no template arguments in main.cpp.
 */
template<
    typename ImuT  = ImuSensor<TrueState>,
    typename GnssT = GpsSensor<TrueState>,
    typename BaroT = BaroSensor<TrueState>,
    typename MagT  = MagSensor<TrueState>
>
class QuadrotorSystemT : public CompositeBlock {
public:
    QuadrotorSystemT(
        const std::string&                    name,
        std::unique_ptr<ImuT>                 imu,
        std::unique_ptr<GnssT>                gnss,
        std::unique_ptr<BaroT>                baro,
        std::unique_ptr<MagT>                 mag,
        std::unique_ptr<shared::INavObserver> observer,
        const QuadrotorDynamics::Params&      dynamics_params,
        const AttitudePidController::Params&  controller_params  = {},
        const GnssOrigin&                     origin             = {},
        uint32_t sensor_period_us   = 1000,
        uint32_t control_period_us  = 1000,
        uint32_t dynamics_period_us = 1000
    )
        : CompositeBlock(name)
        , origin_(origin)
    {
        // Add children in execution order so each sees the freshest upstream data.
        imu_        = add_child(std::move(imu));
        gnss_       = add_child(std::move(gnss));
        baro_       = add_child(std::move(baro));
        mag_        = add_child(std::move(mag));
        ekf_        = add_child(std::make_unique<QuadrotorEkfBlock>(
                          name + "_ekf", std::move(observer), sensor_period_us));
        alt_hold_   = add_child(std::make_unique<AltitudeHoldBlock<NavigationState>>(
                          name + "_alt_hold", control_period_us));
        controller_ = add_child(std::make_unique<AttitudePidController>(
                          name + "_controller", controller_params, control_period_us));
        dynamics_   = add_child(std::make_unique<QuadrotorDynamics>(
                          name + "_dynamics", dynamics_params, dynamics_period_us));

        // The system owns the GNSS origin and pushes it to every block whose
        // local-NED ↔ geodetic conversion depends on it. Keeping it system-
        // scoped (rather than per-block defaults) is what lets sensors and
        // EKF stay consistent.
        gnss_->set_origin(origin_);
        ekf_->set_origin(origin_);
        baro_->set_ground_alt_m(origin_.alt_m);

        // Sensors read from dynamics (previous-step output — correct one-step delay).
        connect(dynamics_->output(), imu_->input());
        connect(dynamics_->output(), gnss_->input());
        connect(dynamics_->output(), baro_->input());
        connect(dynamics_->output(), mag_->input());

        // Sensor outputs feed the EKF.
        connect(imu_->output(),  ekf_->imu_input());
        connect(gnss_->output(), ekf_->gnss_input());
        connect(baro_->output(), ekf_->baro_input());
        connect(mag_->output(),  ekf_->mag_input());

        // EKF estimate drives altitude hold and attitude controller.
        connect(ekf_->output(),      alt_hold_->input());
        connect(ekf_->output(),      controller_->state_input());
        connect(alt_hold_->output(), controller_->thrust_input());

        // Controller efforts drive dynamics.
        connect(controller_->output(), dynamics_->input());
    }

    /**
     * Call after set_params() on dynamics, before running the simulation.
     * Resets dynamics to init, initialises the EKF from the same state,
     * syncs the baro ground-altitude reference, and seeds the alt-hold setpoint.
     */
    void initialize(const TrueState& init) {
        dynamics_->reset(init);
        ekf_->initialize(init);
        alt_hold_->set_setpoint_m(origin_.alt_m - init.position.z());
    }

    const GnssOrigin& origin() const { return origin_; }

    // ----------------------------------------------------------------
    // Component accessors — use these for per-block configuration and
    // logging in the sim harness.
    // ----------------------------------------------------------------
    ImuT&                               imu()        { return *imu_; }
    GnssT&                              gnss()       { return *gnss_; }
    BaroT&                              baro()       { return *baro_; }
    MagT&                               mag()        { return *mag_; }
    QuadrotorEkfBlock&                  ekf()        { return *ekf_; }
    AltitudeHoldBlock<NavigationState>& alt_hold()   { return *alt_hold_; }
    AttitudePidController&              controller() { return *controller_; }
    QuadrotorDynamics&                  dynamics()   { return *dynamics_; }

    // ----------------------------------------------------------------
    // Forwarded disturbance ports for test harnesses.
    // ----------------------------------------------------------------
    InputPort<NedForce>&   disturbance_force_input()  { return dynamics_->disturbance_input(); }
    InputPort<BodyTorque>& disturbance_torque_input() { return dynamics_->disturbance_torque_input(); }

private:
    GnssOrigin                           origin_;
    ImuT*                                imu_        = nullptr;
    GnssT*                               gnss_       = nullptr;
    BaroT*                               baro_       = nullptr;
    MagT*                                mag_        = nullptr;
    QuadrotorEkfBlock*                   ekf_        = nullptr;
    AltitudeHoldBlock<NavigationState>*  alt_hold_   = nullptr;
    AttitudePidController*               controller_ = nullptr;
    QuadrotorDynamics*                   dynamics_   = nullptr;
};

// Convenience alias for the standard simulation-sensor configuration.
using QuadrotorSystem = QuadrotorSystemT<>;

} // namespace quadcopter
} // namespace sim
