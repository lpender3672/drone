#pragma once

#include "../core/block.hpp"
#include "../data/gnss_origin.hpp"
#include "../data/sensor_reading.hpp"
#include "state.hpp"
#include "../../../shared/blocks/quadrotor_ekf.hpp"

// The EKF block moved to shared/blocks/quadrotor_ekf.hpp as a template so
// the same class works on both sim (with sim's ImuData/GnssData/BaroData/
// MagData wrappers, which add InterBlockData<> for logging) and embedded
// (with bare sensors::*Measurement types). This header preserves the old
// `sim::quadcopter::QuadrotorEkfBlock` typedef so existing call sites
// compile unchanged.

namespace sim {
namespace quadcopter {

using QuadrotorEkfBlock = shared::QuadrotorEkfBlockT<
    sim::ImuData,
    sim::GnssData,
    sim::BaroData,
    sim::MagData,
    sim::quadcopter::NavigationState
>;

} // namespace quadcopter
} // namespace sim
