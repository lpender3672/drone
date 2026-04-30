#pragma once

#include "../../../shared/data/gnss_origin.hpp"

// Moved to shared/ for cross-target use. This shim re-exports the type
// under `namespace sim` so existing sim code that says `sim::GnssOrigin`
// keeps compiling unchanged.
namespace sim {
using shared::GnssOrigin;
} // namespace sim
