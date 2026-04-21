#pragma once

#include <cstdint>
#include <string>
#include <array>
#include "../core/interblock_data.hpp"


namespace sim {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Quat = Eigen::Quaterniond;

/**
 * Scalar signal (single value).
 */
class Scalar : public InterBlockData<1> {
public:
    Scalar() = default;
    Scalar(double v, uint64_t timestamp_us = 0) : InterBlockData(timestamp_us) { data_[0] = v; }

    double value() const { return data_[0]; }
    void set_value(double v) { data_[0] = v; }

    std::string type_name() const override { return "Scalar"; }
};

/**
 * Motor output: [omega, thrust, torque]
 */
class MotorOutput : public InterBlockData<3> {
public:
    MotorOutput() = default;

    double omega() const { return data_[0]; }
    double thrust() const { return data_[1]; }
    double torque() const { return data_[2]; }

    void set_omega(double v) { data_[0] = v; }
    void set_thrust(double v) { data_[1] = v; }
    void set_torque(double v) { data_[2] = v; }

    std::string type_name() const override { return "MotorOutput"; }
};

/**
 * PID input: [setpoint, measurement]
 */
class PidInput : public InterBlockData<2> {
public:
    PidInput() = default;
    PidInput(double sp, double meas, uint64_t timestamp_us = 0) : InterBlockData(timestamp_us) {
        data_[0] = sp;
        data_[1] = meas;
    }

    double setpoint() const { return data_[0]; }
    double measurement() const { return data_[1]; }

    void set_setpoint(double v) { data_[0] = v; }
    void set_measurement(double v) { data_[1] = v; }

    std::string type_name() const override { return "PidInput"; }
};

/**
 * Base kinematic state: [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r] = 13 elements
 */
class StateData : public InterBlockData<13> {
public:
    StateData() {
        // Identity quaternion
        data_[6] = 1.0;  // qw
    }

    // Position
    Vec3 position() const { return Vec3(data_[0], data_[1], data_[2]); }
    void set_position(const Vec3& p) { data_[0] = p.x(); data_[1] = p.y(); data_[2] = p.z(); }

    double x() const { return data_[0]; }
    double y() const { return data_[1]; }
    double z() const { return data_[2]; }

    // Velocity
    Vec3 velocity() const { return Vec3(data_[3], data_[4], data_[5]); }
    void set_velocity(const Vec3& v) { data_[3] = v.x(); data_[4] = v.y(); data_[5] = v.z(); }

    double vx() const { return data_[3]; }
    double vy() const { return data_[4]; }
    double vz() const { return data_[5]; }

    // Attitude (quaternion stored as w, x, y, z)
    Quat attitude() const { return Quat(data_[6], data_[7], data_[8], data_[9]); }
    void set_attitude(const Quat& q) {
        data_[6] = q.w();
        data_[7] = q.x();
        data_[8] = q.y();
        data_[9] = q.z();
    }

    // Angular velocity
    Vec3 angular_velocity() const { return Vec3(data_[10], data_[11], data_[12]); }
    void set_angular_velocity(const Vec3& w) { data_[10] = w.x(); data_[11] = w.y(); data_[12] = w.z(); }

    double p() const { return data_[10]; }
    double q() const { return data_[11]; }
    double r() const { return data_[12]; }

    // Derived quantities
    Vec3 euler_angles() const {
        Mat3 R = attitude().toRotationMatrix();
        double roll = std::atan2(R(2, 1), R(2, 2));
        double pitch = std::asin(-std::clamp(R(2, 0), -1.0, 1.0));
        double yaw = std::atan2(R(1, 0), R(0, 0));
        return Vec3(roll, pitch, yaw);
    }

    void set_from_euler(double roll, double pitch, double yaw) {
        Quat q = Eigen::AngleAxisd(yaw, Vec3::UnitZ())
               * Eigen::AngleAxisd(pitch, Vec3::UnitY())
               * Eigen::AngleAxisd(roll, Vec3::UnitX());
        set_attitude(q);
    }

    Mat3 R_nb() const { return attitude().toRotationMatrix(); }
    Mat3 R_bn() const { return attitude().toRotationMatrix().transpose(); }

    void normalise_quaternion() {
        Quat q = attitude();
        q.normalize();
        set_attitude(q);
    }

    std::string type_name() const override { return "StateData"; }
};

/**
 * True state: base state + accelerations [accel(3), specific_force(3), angular_accel(3)] = 22 elements
 */
class TrueStateData : public InterBlockData<22> {
public:
    TrueStateData() {
        data_[6] = 1.0;  // qw identity
    }

    // Base state accessors (same indices as StateData)
    Vec3 position() const { return Vec3(data_[0], data_[1], data_[2]); }
    void set_position(const Vec3& p) { data_[0] = p.x(); data_[1] = p.y(); data_[2] = p.z(); }

    Vec3 velocity() const { return Vec3(data_[3], data_[4], data_[5]); }
    void set_velocity(const Vec3& v) { data_[3] = v.x(); data_[4] = v.y(); data_[5] = v.z(); }

    Quat attitude() const { return Quat(data_[6], data_[7], data_[8], data_[9]); }
    void set_attitude(const Quat& q) {
        data_[6] = q.w();
        data_[7] = q.x();
        data_[8] = q.y();
        data_[9] = q.z();
    }

    Vec3 angular_velocity() const { return Vec3(data_[10], data_[11], data_[12]); }
    void set_angular_velocity(const Vec3& w) { data_[10] = w.x(); data_[11] = w.y(); data_[12] = w.z(); }

    // Extended state
    Vec3 acceleration() const { return Vec3(data_[13], data_[14], data_[15]); }
    void set_acceleration(const Vec3& a) { data_[13] = a.x(); data_[14] = a.y(); data_[15] = a.z(); }

    Vec3 specific_force() const { return Vec3(data_[16], data_[17], data_[18]); }
    void set_specific_force(const Vec3& f) { data_[16] = f.x(); data_[17] = f.y(); data_[18] = f.z(); }

    Vec3 angular_acceleration() const { return Vec3(data_[19], data_[20], data_[21]); }
    void set_angular_acceleration(const Vec3& a) { data_[19] = a.x(); data_[20] = a.y(); data_[21] = a.z(); }

    // Derived quantities
    Vec3 euler_angles() const {
        Mat3 R = attitude().toRotationMatrix();
        double roll = std::atan2(R(2, 1), R(2, 2));
        double pitch = std::asin(-std::clamp(R(2, 0), -1.0, 1.0));
        double yaw = std::atan2(R(1, 0), R(0, 0));
        return Vec3(roll, pitch, yaw);
    }

    void set_from_euler(double roll, double pitch, double yaw) {
        Quat q = Eigen::AngleAxisd(yaw, Vec3::UnitZ())
               * Eigen::AngleAxisd(pitch, Vec3::UnitY())
               * Eigen::AngleAxisd(roll, Vec3::UnitX());
        set_attitude(q);
    }

    Mat3 R_nb() const { return attitude().toRotationMatrix(); }

    void normalise_quaternion() {
        Quat q = attitude();
        q.normalize();
        set_attitude(q);
    }

    // Convert to base state (for observer passthrough)
    StateData to_state_data() const {
        StateData s;
        s.set_position(position());
        s.set_velocity(velocity());
        s.set_attitude(attitude());
        s.set_angular_velocity(angular_velocity());
        s.set_timestamp(timestamp());
        return s;
    }

    std::string type_name() const override { return "TrueStateData"; }
};

/**
 * Observed state: base state + validity + uncertainties = 17 elements
 * [pos(3), vel(3), quat(4), omega(3), valid, pos_std, vel_std, att_std]
 */
class ObservedStateData : public InterBlockData<17> {
public:
    ObservedStateData() {
        data_[6] = 1.0;  // qw identity
    }

    // Base state accessors
    Vec3 position() const { return Vec3(data_[0], data_[1], data_[2]); }
    void set_position(const Vec3& p) { data_[0] = p.x(); data_[1] = p.y(); data_[2] = p.z(); }

    Vec3 velocity() const { return Vec3(data_[3], data_[4], data_[5]); }
    void set_velocity(const Vec3& v) { data_[3] = v.x(); data_[4] = v.y(); data_[5] = v.z(); }

    Quat attitude() const { return Quat(data_[6], data_[7], data_[8], data_[9]); }
    void set_attitude(const Quat& q) {
        data_[6] = q.w();
        data_[7] = q.x();
        data_[8] = q.y();
        data_[9] = q.z();
    }

    Vec3 angular_velocity() const { return Vec3(data_[10], data_[11], data_[12]); }
    void set_angular_velocity(const Vec3& w) { data_[10] = w.x(); data_[11] = w.y(); data_[12] = w.z(); }

    // Estimation quality
    bool valid() const { return data_[13] > 0.5; }
    void set_valid(bool v) { data_[13] = v ? 1.0 : 0.0; }

    double position_stddev() const { return data_[14]; }
    void set_position_stddev(double v) { data_[14] = v; }

    double velocity_stddev() const { return data_[15]; }
    void set_velocity_stddev(double v) { data_[15] = v; }

    double attitude_stddev() const { return data_[16]; }
    void set_attitude_stddev(double v) { data_[16] = v; }

    // Derived quantities
    Vec3 euler_angles() const {
        Mat3 R = attitude().toRotationMatrix();
        double roll = std::atan2(R(2, 1), R(2, 2));
        double pitch = std::asin(-std::clamp(R(2, 0), -1.0, 1.0));
        double yaw = std::atan2(R(1, 0), R(0, 0));
        return Vec3(roll, pitch, yaw);
    }

    Mat3 R_nb() const { return attitude().toRotationMatrix(); }

    void normalise_quaternion() {
        Quat q = attitude();
        q.normalize();
        set_attitude(q);
    }

    std::string type_name() const override { return "ObservedStateData"; }
};

/**
 * External torque in body frame [Tx, Ty, Tz] in N·m.
 */
class BodyTorque : public InterBlockData<3> {
public:
    BodyTorque() = default;
    explicit BodyTorque(const Vec3& t) { data_[0]=t.x(); data_[1]=t.y(); data_[2]=t.z(); }

    Vec3 torque() const { return Vec3(data_[0], data_[1], data_[2]); }
    void set_torque(const Vec3& t) { data_[0]=t.x(); data_[1]=t.y(); data_[2]=t.z(); }

    std::string type_name() const override { return "BodyTorque"; }
};

/**
 * External force vector in NED frame [Fx, Fy, Fz] in Newtons.
 */
class NedForce : public InterBlockData<3> {
public:
    NedForce() = default;
    explicit NedForce(const Vec3& f) { data_[0]=f.x(); data_[1]=f.y(); data_[2]=f.z(); }

    Vec3 force() const { return Vec3(data_[0], data_[1], data_[2]); }
    void set_force(const Vec3& f) { data_[0]=f.x(); data_[1]=f.y(); data_[2]=f.z(); }

    std::string type_name() const override { return "NedForce"; }
};

/**
 * Empty signal for blocks with no input.
 */
class NoInput : public InterBlockData<0> {
public:
    NoInput() = default;
    std::string type_name() const override { return "NoInput"; }
};

} // namespace sim