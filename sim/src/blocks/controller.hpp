#pragma once

#include "../../../shared/blocks/controller.hpp"

// ControllerBlock moved to shared/blocks/controller.hpp for cross-target use.
namespace sim {
template<typename NavigationStateT, typename ReferenceT, typename EffortsT>
using ControllerBlock = shared::ControllerBlock<NavigationStateT, ReferenceT, EffortsT>;
} // namespace sim
