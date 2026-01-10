#pragma once

#include "../core/inter_block_data.hpp"

namespace sim {

/**
 * Reference/setpoint for the controller.
 * Can represent different control modes via the mode field.
 */
struct Reference : public InterBlockData {
    enum class Mode {
        RATE,           // Direct angular rate control (p, q, r) + thrust
        ATTITUDE,       // Attitude angles (roll, pitch, yaw) + thrust  
        VELOCITY,       // Velocity setpoint (vx, vy, vz) + yaw
        POSITION        // Position setpoint (x, y, z) + yaw
    };

    Mode mode = Mode::ATTITUDE;

    // Attitude mode: desired angles [rad] (roll, pitch, yaw)
    Vec3 attitude = Vec3::Zero();

    // Rate mode: desired angular rates [rad/s] (p, q, r)
    Vec3 angular_rate = Vec3::Zero();

    // Position mode: desired position [m] in NED
    Vec3 position = Vec3::Zero();

    // Velocity mode: desired velocity [m/s] in NED
    Vec3 velocity = Vec3::Zero();

    // Thrust command (interpretation depends on mode)
    // For RATE/ATTITUDE: normalised collective thrust [0, 1]
    // For VELOCITY/POSITION: desired vertical velocity or altitude
    double thrust = 0.0;

    Reference() = default;
    explicit Reference(double timestamp_s) : InterBlockData(timestamp_s) {}

    std::string type_name() const override { return "Reference"; }
    
    InterBlockData* clone() const override {
        return new Reference(*this);
    }

    // Convenience accessors for attitude
    double roll() const { return attitude.x(); }
    double pitch() const { return attitude.y(); }
    double yaw() const { return attitude.z(); }
    
    void set_roll(double v) { attitude.x() = v; }
    void set_pitch(double v) { attitude.y() = v; }
    void set_yaw(double v) { attitude.z() = v; }

    // Convenience accessors for angular rate
    double p() const { return angular_rate.x(); }
    double q() const { return angular_rate.y(); }
    double r() const { return angular_rate.z(); }
};

} // namespace sim
