#pragma once

#include "../../../shared/blocks/pid.hpp"

// PidBlock moved to shared/blocks/pid.hpp for cross-target use. Sim code
// keeps using `sim::PidBlock` / `sim::PidParams` via these aliases.
namespace sim {
using shared::PidParams;
using shared::PidBlock;
} // namespace sim
