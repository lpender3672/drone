#pragma once

#include "../../../shared/blocks/attitude_controller.hpp"
#include "state.hpp"

// AttitudePidController is the default specialization on bare
// shared::NavigationState — same type universe as embedded.
namespace sim {
namespace quadcopter {

using AttitudePidController = shared::AttitudePidController;

} // namespace quadcopter
} // namespace sim
