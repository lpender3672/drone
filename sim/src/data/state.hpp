#pragma once

#include "../core/inter_block_data.hpp"

namespace sim {

/**
 * Full drone state vector.
 * Uses NED frame for position/velocity, Eigen quaternion for attitude.
 */
struct State : public InterBlockData {
    // Position in NED frame [m]
    Vec3 position = Vec3::Zero();
    
    // Velocity in NED frame [m/s]
    Vec3 velocity = Vec3::Zero();
    
    // Attitude quaternion (Eigen uses [x,y,z,w] internally but we use w,x,y,z convention)
    Quat attitude = Quat::Identity();
    
    // Angular velocity in body frame [rad/s]
    Vec3 angular_velocity = Vec3::Zero();

    State() = default;
    explicit State(double timestamp_s) : InterBlockData(timestamp_s) {}

    std::string type_name() const override { return "State"; }
    
    InterBlockData* clone() const override {
        return new State(*this);
    }

    // Convenience accessors for position
    double x() const { return position.x(); }
    double y() const { return position.y(); }
    double z() const { return position.z(); }

    // Convenience accessors for velocity
    double vx() const { return velocity.x(); }
    double vy() const { return velocity.y(); }
    double vz() const { return velocity.z(); }

    // Convenience accessors for angular velocity
    double p() const { return angular_velocity.x(); }
    double q() const { return angular_velocity.y(); }
    double r() const { return angular_velocity.z(); }

    // Get Euler angles from quaternion [rad] (roll, pitch, yaw)
    Vec3 euler_angles() const {
        // Convert to rotation matrix then extract Euler angles (ZYX convention)
        Mat3 R = attitude.toRotationMatrix();
        
        double roll = std::atan2(R(2,1), R(2,2));
        double pitch = std::asin(-std::clamp(R(2,0), -1.0, 1.0));
        double yaw = std::atan2(R(1,0), R(0,0));
        
        return Vec3(roll, pitch, yaw);
    }

    // Set quaternion from Euler angles [rad] (roll, pitch, yaw)
    void set_from_euler(double roll, double pitch, double yaw) {
        // ZYX convention: yaw * pitch * roll
        attitude = Eigen::AngleAxisd(yaw, Vec3::UnitZ())
                 * Eigen::AngleAxisd(pitch, Vec3::UnitY())
                 * Eigen::AngleAxisd(roll, Vec3::UnitX());
    }

    void set_from_euler(const Vec3& rpy) {
        set_from_euler(rpy.x(), rpy.y(), rpy.z());
    }

    // Rotation matrix from body to NED
    Mat3 R_nb() const {
        return attitude.toRotationMatrix();
    }

    // Rotation matrix from NED to body
    Mat3 R_bn() const {
        return attitude.toRotationMatrix().transpose();
    }

    // Normalise quaternion
    void normalise_quaternion() {
        attitude.normalize();
    }
};

} // namespace sim
