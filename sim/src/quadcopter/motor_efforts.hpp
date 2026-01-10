#pragma once

#include "../data/control_efforts.hpp"

namespace sim {
namespace quadcopter {

/**
 * Motor command outputs from controller.
 * Values are normalised [0, 1] representing throttle fraction.
 * Motor numbering: 0=front-right, 1=back-left, 2=front-left, 3=back-right
 * (X configuration, CW/CCW alternating)
 */
struct MotorEfforts : public ControlEfforts<4> {
    using Base = ControlEfforts<4>;

    MotorEfforts() = default;
    explicit MotorEfforts(double timestamp_s) : Base(timestamp_s) {}
    
    MotorEfforts(double m0, double m1, double m2, double m3, double timestamp_s = 0.0)
        : Base(Vec4(m0, m1, m2, m3), timestamp_s)
    {}

    explicit MotorEfforts(const Vec4& e, double timestamp_s = 0.0)
        : Base(e, timestamp_s)
    {}

    std::string type_name() const override { return "MotorEfforts"; }
    
    InterBlockData* clone() const override {
        return new MotorEfforts(*this);
    }
};

} // namespace quadcopter
} // namespace sim
