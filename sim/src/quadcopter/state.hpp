#pragma once

#include "../../../shared/types/state.h"

// Quadrotor-specific state types are just the shared rigid-body kinematic
// states — quadrotors don't need any state extensions beyond what every
// flying rigid body has. Future vehicle types (fixed-wing, helicopter, VTOL)
// that need extra fields (airspeed, sideslip, rotor RPM) will define their
// own derived state structs in their own vehicle module — this is the slot
// where that variation belongs, but quadrotor doesn't need it.

namespace sim {
namespace quadcopter {

using shared::TrueState;
using shared::NavigationState;

} // namespace quadcopter
} // namespace sim
