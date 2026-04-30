#pragma once

#include "../../../shared/core/interblock_data.hpp"

// The block runtime moved to shared/core/ so the embedded target can use the
// same primitives. This header re-exports the names under `namespace sim` so
// existing sim code that says `sim::IInterBlockData` keeps compiling unchanged.
namespace sim {
using shared::IInterBlockData;
template<int N>
using InterBlockData = shared::InterBlockData<N>;
} // namespace sim
