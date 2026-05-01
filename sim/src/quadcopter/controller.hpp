#pragma once

#include "../../../shared/blocks/attitude_controller.hpp"
#include "state.hpp"

// AttitudePidController moved to shared/blocks/attitude_controller.hpp as a
// template. Sim instantiates it with sim's NavigationState wrapper (adds an
// InterBlockData<17> sub-data for future per-block logging); embedded uses
// the default specialization with bare shared::NavigationState.
namespace sim {
namespace quadcopter {

using AttitudePidController = shared::AttitudePidControllerT<NavigationState>;

} // namespace quadcopter
} // namespace sim
