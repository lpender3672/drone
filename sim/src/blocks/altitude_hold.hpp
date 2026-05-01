#pragma once

#include "../../../shared/blocks/altitude_hold.hpp"

// AltitudeHoldBlock moved to shared/blocks/altitude_hold.hpp.
namespace sim {
template<typename NavigationStateT = shared::NavigationState>
using AltitudeHoldBlock = shared::AltitudeHoldBlock<NavigationStateT>;
} // namespace sim
