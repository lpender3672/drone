#pragma once

#include <cstdint>
#include <string>
#include <array>
#include "../data/state.hpp"


namespace sim {


/**
 * Motor efforts: [m1, m2, m3, m4]
 */
class MotorEfforts : public InterBlockData<4> {
public:
    MotorEfforts() = default;

    

    std::string type_name() const override { return "MotorEfforts"; }
};

class AttitudeReference : public InterBlockData<4> {
public:
    AttitudeReference() = default;

    double roll() const { return data_[0]; }
    double pitch() const { return data_[1]; }
    double yaw() const { return data_[2]; }

    Vec3 attitude() const { return Vec3(data_[0], data_[1], data_[2]); }

    void set_roll(double v) { data_[0] = v; }
    void set_pitch(double v) { data_[1] = v; }
    void set_yaw(double v) { data_[2] = v; }

    double thrust() const { return data_[3]; }
    void set_thrust(double v) { data_[3] = v; }

    std::string type_name() const override { return "AttitudeReference"; }
};

// default true and observed state types

} // namespace sim