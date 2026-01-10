#pragma once

#include "../core/inter_block_data.hpp"
#include "../../../shared/types/state.h"

namespace sim {

/**
 * Full drone state vector for simulation.
 * Inherits from shared::StateBase for compatibility with IObserver/IController.
 * Also inherits InterBlockData for sim block communication.
 */
struct State : public InterBlockData, public shared::StateBase {

    State() = default;
    explicit State(double timestamp_s) : InterBlockData(timestamp_s) {}
    
    // Construct from shared::StateBase
    explicit State(const shared::StateBase& base, double timestamp_s = 0.0)
        : InterBlockData(timestamp_s)
        , shared::StateBase(base)
    {}
    
    // Construct from shared::StateWithBiases (strips biases)
    explicit State(const shared::StateWithBiases& with_biases, double timestamp_s = 0.0)
        : InterBlockData(timestamp_s)
        , shared::StateBase(static_cast<const shared::StateBase&>(with_biases))
    {}

    std::string type_name() const override { return "State"; }
    
    InterBlockData* clone() const override {
        return new State(*this);
    }

    // Normalise quaternion
    void normalise_quaternion() {
        attitude.normalize();
    }
};

} // namespace sim
