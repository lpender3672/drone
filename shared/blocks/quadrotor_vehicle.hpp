#pragma once

#include <memory>

#include "../core/block.hpp"
#include "../data/gnss_origin.hpp"
#include "../data/state.hpp"
#include "../types/state.h"
#include "altitude_hold.hpp"
#include "attitude_controller.hpp"
#include "quadrotor_ekf.hpp"

namespace shared {

/**
 * Quadrotor vehicle as a CompositeBlock — runs on both sim and embedded.
 *
 * Children (ticked in this order each update):
 *   1. EKF             (sensor inputs → NavigationState output)
 *   2. AltitudeHold    (NavigationState → thrust scalar)
 *   3. AttitudeController  (NavigationState + AttitudeReference + thrust → MotorEfforts)
 *
 * External interface — forwards through to the children's ports so callers
 * connect() directly to/from the relevant child:
 *   - imu_input(), gnss_input(), baro_input(), mag_input()  → EKF inputs
 *   - reference_input()                                     → controller's attitude reference
 *   - motor_output()                                        → controller's motor efforts
 *
 * Sensors are NOT children of the vehicle — owners (sim's plant, embedded's
 * main) hold them and connect() their outputs into the vehicle's inputs.
 * Same shape on both targets: only who owns the sensors differs.
 *
 * Templated on the EKF / AltHold / Controller types so the same vehicle class
 * works for sim (NavigationState wrapper with InterBlockData<> for logging)
 * and embedded (bare shared::NavigationState).
 */
template<
    typename TEkf        = QuadrotorEkfBlockT<>,
    typename TAltHold    = AltitudeHoldBlock<NavigationState>,
    typename TController = AttitudePidControllerT<NavigationState>
>
class QuadrotorVehicleT : public CompositeBlock {
public:
    QuadrotorVehicleT(
        const std::string&                     name,
        std::unique_ptr<INavObserver>          observer,
        const typename TController::Params&    controller_params = {},
        const GnssOrigin&                      origin            = {},
        uint32_t sensor_period_us  = 1000,
        uint32_t control_period_us = 1000
    )
        : CompositeBlock(name)
        , origin_(origin)
    {
        ekf_        = add_child(std::make_unique<TEkf>(
                          name + "_ekf", std::move(observer), sensor_period_us));
        alt_hold_   = add_child(std::make_unique<TAltHold>(
                          name + "_alt_hold", control_period_us));
        controller_ = add_child(std::make_unique<TController>(
                          name + "_controller", controller_params, control_period_us));

        // Wire the internal cascade: EKF → alt-hold (altitude loop)
        //                           EKF → controller (state input)
        //                           alt-hold → controller (thrust input)
        connect(ekf_->output(),      alt_hold_->input());
        connect(ekf_->output(),      controller_->state_input());
        connect(alt_hold_->output(), controller_->thrust_input());

        ekf_->set_origin(origin_);
    }

    /**
     * Initialise the EKF from a known truth state and seed the alt-hold
     * setpoint from the same. The system harness (sim plant or embedded
     * bring-up) calls this once before running.
     */
    void initialize(const TrueState& init) {
        ekf_->initialize(init);
        alt_hold_->set_setpoint_m(origin_.alt_m - init.position.z());
    }

    // External ports forwarded to the children.
    auto& imu_input()        { return ekf_->imu_input(); }
    auto& gnss_input()       { return ekf_->gnss_input(); }
    auto& baro_input()       { return ekf_->baro_input(); }
    auto& mag_input()        { return ekf_->mag_input(); }
    auto& reference_input()  { return controller_->reference_input(); }
    auto& motor_output()     { return controller_->output(); }

    // Component accessors for runtime configuration / per-block tuning.
    TEkf&        ekf()        { return *ekf_; }
    TAltHold&    alt_hold()   { return *alt_hold_; }
    TController& controller() { return *controller_; }

    const GnssOrigin& origin() const { return origin_; }

private:
    GnssOrigin    origin_;
    TEkf*         ekf_        = nullptr;
    TAltHold*     alt_hold_   = nullptr;
    TController*  controller_ = nullptr;
};

// Default instantiation used by both sim and embedded — bare measurement
// types and shared::NavigationState. Sim wraps it inside a QuadrotorPlant
// (sim/src/quadcopter/plant.hpp) that adds dynamics + sensors; embedded
// constructs it directly (ekf-teensy/src/main.cpp).
using QuadrotorVehicle = QuadrotorVehicleT<>;

} // namespace shared
