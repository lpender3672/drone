#pragma once

#include <cstdint>
#include <string>
#include <array>
#include "../data/state.hpp"

// Sim-specific extensions of the shared kinematic states. These add an
// InterBlockData<> sub-data to the shared TrueState/NavigationState so
// the sim's logger can introspect them. Embedded code uses the shared
// types directly and doesn't need these wrappers.

namespace sim {
namespace quadcopter {

class TrueState : public shared::TrueState, public InterBlockData<13> {
public:
    TrueState() = default;

    std::string type_name() const override { return "TrueState"; }
};

class NavigationState : public shared::NavigationState, public InterBlockData<17> {
public:
    NavigationState() = default;
    explicit NavigationState(const shared::TrueState& base) : shared::NavigationState(base) {}

    // Assign just the navigation kinematics from a shared::NavigationState
    // value, leaving the InterBlockData<17> sub-data (used by the logger)
    // untouched. Wraps what would otherwise be a slicing static_cast.
    void assign_nav(const shared::NavigationState& src) {
        static_cast<shared::NavigationState&>(*this) = src;
    }

    std::string type_name() const override { return "NavigationState"; }
};

} // namespace quadcopter
} // namespace sim
