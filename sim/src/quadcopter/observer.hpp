#pragma once

#include "../core/block.hpp"
#include "../data/gnss_origin.hpp"
#include "state.hpp"
#include "../../../shared/blocks/quadrotor_ekf.hpp"

// Sim and embedded both use the canonical sensors::*Measurement input
// types and the bare shared::NavigationState output — there's only one
// type universe for the EKF block now. This typedef just lets sim's
// existing call sites keep saying `sim::quadcopter::QuadrotorEkfBlock`.

namespace sim {
namespace quadcopter {

using QuadrotorEkfBlock = shared::QuadrotorEkfBlock;

} // namespace quadcopter
} // namespace sim
