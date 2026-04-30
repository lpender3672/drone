#pragma once

#include "../../../shared/core/block.hpp"
#include "interblock_data.hpp"

// The block runtime moved to shared/core/ so the embedded target can use the
// same primitives. This header re-exports the names under `namespace sim` so
// existing sim code that says `sim::Block`, `sim::connect`, etc. keeps
// compiling unchanged.
namespace sim {
template<typename T>
using InputPort  = shared::InputPort<T>;
template<typename T>
using OutputPort = shared::OutputPort<T>;
using shared::Block;
using shared::CompositeBlock;
template<typename TIn, typename TOut>
using TypedBlock = shared::TypedBlock<TIn, TOut>;
template<typename TOut>
using SourceBlock = shared::SourceBlock<TOut>;

// connect() reaches the unqualified call sites via ADL on the port types
// (which now live in `shared`), so no using-declaration needed here.
} // namespace sim
