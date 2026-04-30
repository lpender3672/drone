#pragma once
#include "../core/block.hpp"
#include "../data/state.hpp"

namespace sim {

/**
 * Scales a scalar signal by a fixed NED direction vector to produce a force.
 * Connect a WaveformGenerator output → this input → dynamics disturbance port.
 */
class ForceDisturbanceBlock : public TypedBlock<Scalar, NedForce> {
public:
    ForceDisturbanceBlock(const std::string& name,
                          const Vec3& direction,
                          uint32_t update_period_us = 0)
        : TypedBlock<Scalar, NedForce>(name, "scalar", "force", update_period_us)
        , direction_(direction.normalized())
    {}

    bool update(uint64_t current_time_us) override {
        if (!is_due(current_time_us)) return false;
        output_.value.set_force(direction_ * input_.get().value());
        mark_updated(current_time_us);
        return true;
    }

private:
    Vec3 direction_;
};

/**
 * Scales a scalar signal by a fixed body-frame axis to produce a torque disturbance.
 */
class TorqueDisturbanceBlock : public TypedBlock<Scalar, BodyTorque> {
public:
    TorqueDisturbanceBlock(const std::string& name,
                           const Vec3& axis,
                           uint32_t update_period_us = 0)
        : TypedBlock<Scalar, BodyTorque>(name, "scalar", "torque", update_period_us)
        , axis_(axis.normalized())
    {}

    bool update(uint64_t current_time_us) override {
        if (!is_due(current_time_us)) return false;
        output_.value.set_torque(axis_ * input_.get().value());
        mark_updated(current_time_us);
        return true;
    }

private:
    Vec3 axis_;
};

} // namespace sim
