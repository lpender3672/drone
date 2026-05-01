#pragma once

#include "../core/block.hpp"
#include "../data/gnss_origin.hpp"
#include "state.hpp"
#include <sensor_readings.h>
#include "../../../shared/blocks/quadrotor_ekf.hpp"

// The EKF block lives in shared/blocks/quadrotor_ekf.hpp as a template so
// the same class works on both sim and embedded. Sim and embedded now both
// use bare sensors::*Measurement input types — sensor data has a single
// canonical type universe across the codebase. The only sim-specific bit
// is the output-state type, which is sim's NavigationState wrapper (adds
// InterBlockData<> for the future per-block logger).

namespace sim {
namespace quadcopter {

using QuadrotorEkfBlock = shared::QuadrotorEkfBlockT<
    sensors::ImuMeasurement,
    sensors::GnssMeasurement,
    sensors::BaroMeasurement,
    sensors::MagMeasurement,
    sim::quadcopter::NavigationState
>;

} // namespace quadcopter
} // namespace sim
