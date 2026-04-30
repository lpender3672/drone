#pragma once

#include "../../../shared/blocks/attitude_controller.hpp"
#include "state.hpp"

// AttitudePidController moved to shared/blocks/attitude_controller.hpp as a
// template. Sim instantiates it with sim's NavigationState wrapper so the
// existing ImuData/InterBlockData<> logging path is unchanged.
namespace sim {
namespace quadcopter {

using AttitudePidController = shared::AttitudePidControllerT<NavigationState>;

} // namespace quadcopter
} // namespace sim
