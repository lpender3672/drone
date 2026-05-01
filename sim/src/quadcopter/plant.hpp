#pragma once

#include <memory>

#include "../core/block.hpp"
#include "../blocks/sensors_impl.hpp"
#include "../data/gnss_origin.hpp"
#include "dynamics.hpp"
#include "controller.hpp"
#include "observer.hpp"
#include "state.hpp"
#include "../../../shared/blocks/quadrotor_vehicle.hpp"
#include "../../../shared/observer.h"

namespace sim {
namespace quadcopter {

/**
 * Sim-only "plant": the closed-loop system the controller is closing the
 * loop on. Owns a shared::QuadrotorVehicle (sensors-EKF-controller chain
 * that runs on both targets) plus the sim-only dynamics + sensor blocks
 * + disturbance-injection ports needed for closed-loop simulation.
 *
 * Embedded targets construct shared::QuadrotorVehicle directly — there
 * is no Plant on hardware; the real world plays that role.
 */
template<
    typename ImuT  = ImuSensor<TrueState>,
    typename GnssT = GpsSensor<TrueState>,
    typename BaroT = BaroSensor<TrueState>,
    typename MagT  = MagSensor<TrueState>
>
class QuadrotorPlantT : public CompositeBlock {
public:
    // Sim uses the default vehicle specialization — same type universe as
    // embedded. Per-vehicle state extensions (fixed-wing airspeed/sideslip,
    // helicopter rotor RPM) will eventually live in their own vehicle modules
    // and re-instantiate this template; quadrotor doesn't need any.
    using Vehicle = shared::QuadrotorVehicle;

    QuadrotorPlantT(
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
        // Children added in execution order. Vehicle internally schedules
        // its own children (EKF, alt-hold, controller) when its update()
        // ticks; outside of that, each entry here is one tick.
        imu_      = add_child(std::move(imu));
        gnss_     = add_child(std::move(gnss));
        baro_     = add_child(std::move(baro));
        mag_      = add_child(std::move(mag));
        vehicle_  = add_child(std::make_unique<Vehicle>(
                       name + "_vehicle",
                       std::move(observer),
                       controller_params,
                       origin_,
                       sensor_period_us,
                       control_period_us));
        dynamics_ = add_child(std::make_unique<QuadrotorDynamics>(
                       name + "_dynamics", dynamics_params, dynamics_period_us));

        // GNSS origin is system-scoped — the vehicle pushes it into its EKF
        // via its own ctor; the system additionally syncs the GPS sensor
        // and the baro ground reference so they all agree.
        gnss_->set_origin(origin_);
        baro_->set_ground_alt_m(origin_.alt_m);

        // Sensors read from dynamics (previous-step output — correct one-step delay).
        connect(dynamics_->output(), imu_->input());
        connect(dynamics_->output(), gnss_->input());
        connect(dynamics_->output(), baro_->input());
        connect(dynamics_->output(), mag_->input());

        // Sensor outputs feed the vehicle's EKF inputs.
        connect(imu_->output(),  vehicle_->imu_input());
        connect(gnss_->output(), vehicle_->gnss_input());
        connect(baro_->output(), vehicle_->baro_input());
        connect(mag_->output(),  vehicle_->mag_input());

        // Vehicle's motor commands drive the dynamics.
        connect(vehicle_->motor_output(), dynamics_->input());

        // Plant's own composite topology is fixed once all children/edges
        // above are in place. (Vehicle freezes its own internal graph in
        // its ctor.)
        freeze();
    }

    /**
     * Reset dynamics to the init state and ask the vehicle to seed its
     * estimator and alt-hold setpoint from the same.
     */
    void initialize(const TrueState& init) {
        dynamics_->reset(init);
        vehicle_->initialize(init);
    }

    const GnssOrigin& origin() const { return origin_; }

    // Sensor accessors.
    ImuT&  imu()  { return *imu_; }
    GnssT& gnss() { return *gnss_; }
    BaroT& baro() { return *baro_; }
    MagT&  mag()  { return *mag_; }

    // Vehicle / vehicle-internal accessors. The forwarders preserve the
    // existing call-site shape (`system->ekf()`, `system->controller()`).
    Vehicle&                            vehicle()    { return *vehicle_; }
    QuadrotorEkfBlock&                  ekf()        { return vehicle_->ekf(); }
    shared::AltitudeHoldBlock<NavigationState>& alt_hold() { return vehicle_->alt_hold(); }
    AttitudePidController&              controller() { return vehicle_->controller(); }
    QuadrotorDynamics&                  dynamics()   { return *dynamics_; }

    // Forwarded disturbance ports for test harnesses.
    InputPort<NedForce>&   disturbance_force_input()  { return dynamics_->disturbance_input(); }
    InputPort<BodyTorque>& disturbance_torque_input() { return dynamics_->disturbance_torque_input(); }

private:
    GnssOrigin           origin_;
    ImuT*                imu_      = nullptr;
    GnssT*               gnss_     = nullptr;
    BaroT*               baro_     = nullptr;
    MagT*                mag_      = nullptr;
    Vehicle*             vehicle_  = nullptr;
    QuadrotorDynamics*   dynamics_ = nullptr;
};

// Convenience alias for the standard simulation-sensor configuration.
using QuadrotorPlant = QuadrotorPlantT<>;

} // namespace quadcopter
} // namespace sim
