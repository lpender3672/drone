#pragma once

#include "../../../shared/core/interblock_data.hpp"

// The block runtime moved to shared/core/ so the embedded target can use the
// same primitives. This header re-exports the InterBlockData<N> template
// under `namespace sim` so existing sim code keeps compiling unchanged.
// (IInterBlockData was removed in slice 6 — no consumers.)
namespace sim {
template<int N>
using InterBlockData = shared::InterBlockData<N>;
} // namespace sim
