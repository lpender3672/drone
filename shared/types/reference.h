#ifndef SHARED_TYPES_REFERENCE_H
#define SHARED_TYPES_REFERENCE_H

#include <Eigen/Dense>

namespace shared {

// Common Eigen typedefs (same as state.h)
using Vec3 = Eigen::Vector3d;

/**
 * Base reference/setpoint for controller.
 * Contains the minimal fields needed for control.
 * Derived types can add modes and additional fields (e.g., sim::Reference).
 */
struct ReferenceBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Vec3 position = Vec3::Zero();           // Desired position NED [m]
    Vec3 velocity = Vec3::Zero();           // Desired velocity NED [m/s]
    Vec3 attitude = Vec3::Zero();           // Desired attitude [rad] (roll, pitch, yaw)
    Vec3 angular_rate = Vec3::Zero();       // Desired angular rates [rad/s]
    double thrust = 0.0;                    // Normalized thrust [0, 1]
    
    ReferenceBase() = default;
    
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
    
    void set_p(double v) { angular_rate.x() = v; }
    void set_q(double v) { angular_rate.y() = v; }
    void set_r(double v) { angular_rate.z() = v; }
};

} // namespace shared

#endif // SHARED_TYPES_REFERENCE_H
