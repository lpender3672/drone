#ifndef SHARED_TYPES_STATE_H
#define SHARED_TYPES_STATE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace shared {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

/**
 * Base kinematic state.
 * Contains minimal state needed for control and estimation.
 */
struct TrueState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3 position = Vec3::Zero();
    Vec3 velocity = Vec3::Zero();
    Quat attitude = Quat::Identity();
    Vec3 angular_velocity = Vec3::Zero();
    Vec3 linear_accel = Vec3::Zero();  // NED acceleration (m/s²), set by dynamics
    
    TrueState() = default;
    
    double x() const { return position.x(); }
    double y() const { return position.y(); }
    double z() const { return position.z(); }
    
    double vx() const { return velocity.x(); }
    double vy() const { return velocity.y(); }
    double vz() const { return velocity.z(); }
    
    double p() const { return angular_velocity.x(); }
    double q() const { return angular_velocity.y(); }
    double r() const { return angular_velocity.z(); }
    
    Vec3 euler_angles() const {
        Mat3 R = attitude.toRotationMatrix();
        double roll = std::atan2(R(2,1), R(2,2));
        double r20 = R(2,0) < -1.0 ? -1.0 : (R(2,0) > 1.0 ? 1.0 : R(2,0));
        double pitch = std::asin(-r20);
        double yaw = std::atan2(R(1,0), R(0,0));
        return Vec3(roll, pitch, yaw);
    }
    
    void set_from_euler(double roll, double pitch, double yaw) {
        attitude = Eigen::AngleAxisd(yaw, Vec3::UnitZ())
                 * Eigen::AngleAxisd(pitch, Vec3::UnitY())
                 * Eigen::AngleAxisd(roll, Vec3::UnitX());
    }
    
    void set_from_euler(const Vec3& rpy) {
        set_from_euler(rpy.x(), rpy.y(), rpy.z());
    }
    
    Mat3 R_nb() const { return attitude.toRotationMatrix(); }
    Mat3 R_bn() const { return attitude.toRotationMatrix().transpose(); }
    
    void normalise_quaternion() { attitude.normalize(); }
};


/**
 * Output of a navigation filter: vehicle state plus estimated sensor biases.
 * (Standard term in INS/GPS literature — "navigation solution" or "nav state".)
 */
struct NavigationState : public TrueState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid = false;
    Vec3 accel_bias = Vec3::Zero();
    Vec3 gyro_bias = Vec3::Zero();
    double baro_bias = 0.0;

    NavigationState() = default;
    explicit NavigationState(const TrueState& base) : TrueState(base), valid(true) {}
};

} // namespace shared

#endif // SHARED_TYPES_STATE_H