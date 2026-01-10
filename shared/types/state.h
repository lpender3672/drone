#ifndef SHARED_TYPES_STATE_H
#define SHARED_TYPES_STATE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace shared {

// Common Eigen typedefs
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

/**
 * Base state representation for observer output.
 * Contains the minimal state needed for control.
 * Derived types can add additional fields (e.g., biases for EKF).
 */
struct StateBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Vec3 position = Vec3::Zero();           // Position in NED frame [m]
    Vec3 velocity = Vec3::Zero();           // Velocity in NED frame [m/s]
    Quat attitude = Quat::Identity();       // Attitude quaternion (body to NED)
    Vec3 angular_velocity = Vec3::Zero();   // Angular velocity in body frame [rad/s]
    
    StateBase() = default;
    
    // Convenience accessors
    double x() const { return position.x(); }
    double y() const { return position.y(); }
    double z() const { return position.z(); }
    
    double vx() const { return velocity.x(); }
    double vy() const { return velocity.y(); }
    double vz() const { return velocity.z(); }
    
    double p() const { return angular_velocity.x(); }
    double q() const { return angular_velocity.y(); }
    double r() const { return angular_velocity.z(); }
    
    // Euler angles from quaternion [rad] (roll, pitch, yaw) - ZYX convention
    Vec3 euler_angles() const {
        Mat3 R = attitude.toRotationMatrix();
        double roll = std::atan2(R(2,1), R(2,2));
        double pitch = std::asin(-std::clamp(R(2,0), -1.0, 1.0));
        double yaw = std::atan2(R(1,0), R(0,0));
        return Vec3(roll, pitch, yaw);
    }
    
    // Set quaternion from Euler angles [rad] (roll, pitch, yaw)
    void set_from_euler(double roll, double pitch, double yaw) {
        attitude = Eigen::AngleAxisd(yaw, Vec3::UnitZ())
                 * Eigen::AngleAxisd(pitch, Vec3::UnitY())
                 * Eigen::AngleAxisd(roll, Vec3::UnitX());
    }
    
    void set_from_euler(const Vec3& rpy) {
        set_from_euler(rpy.x(), rpy.y(), rpy.z());
    }
    
    // Rotation matrix from body to NED
    Mat3 R_nb() const { return attitude.toRotationMatrix(); }
    
    // Rotation matrix from NED to body
    Mat3 R_bn() const { return attitude.toRotationMatrix().transpose(); }
};

/**
 * Extended state with bias estimates for EKF output.
 * Controller can use base fields; logger/diagnostics can access biases.
 */
struct StateWithBiases : public StateBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Vec3 accel_bias = Vec3::Zero();     // Accelerometer bias [m/s^2]
    Vec3 gyro_bias = Vec3::Zero();      // Gyroscope bias [rad/s]
    double baro_bias = 0.0;             // Barometer altitude bias [m]
    
    StateWithBiases() = default;
    
    // Construct from base state (biases remain zero)
    explicit StateWithBiases(const StateBase& base)
        : StateBase(base)
    {}
};

} // namespace shared

#endif // SHARED_TYPES_STATE_H
