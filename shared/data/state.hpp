#pragma once

#include <cstdint>
#include <array>
#include "../core/interblock_data.hpp"
#include "../types/state.h"

namespace shared {

// Eigen aliases not already provided by shared/types/state.h.
using Vec2 = Eigen::Vector2d;
using Vec4 = Eigen::Vector4d;
using Mat4 = Eigen::Matrix4d;

// Scalar signal (single value).
class Scalar : public InterBlockData<1> {
public:
    Scalar() = default;
    explicit Scalar(double v) { data_[0] = v; }

    double value() const { return data_[0]; }
    void set_value(double v) { data_[0] = v; }
};

// Motor output: [omega, thrust, torque]
class MotorOutput : public InterBlockData<3> {
public:
    MotorOutput() = default;

    double omega() const { return data_[0]; }
    double thrust() const { return data_[1]; }
    double torque() const { return data_[2]; }

    void set_omega(double v) { data_[0] = v; }
    void set_thrust(double v) { data_[1] = v; }
    void set_torque(double v) { data_[2] = v; }
};

// PID input: [setpoint, measurement]
class PidInput : public InterBlockData<2> {
public:
    PidInput() = default;
    PidInput(double sp, double meas) {
        data_[0] = sp;
        data_[1] = meas;
    }

    double setpoint() const { return data_[0]; }
    double measurement() const { return data_[1]; }

    void set_setpoint(double v) { data_[0] = v; }
    void set_measurement(double v) { data_[1] = v; }
};

// Motor efforts: [m1, m2, m3, m4]
class MotorEfforts : public InterBlockData<4> {
public:
    MotorEfforts() = default;
};

// Attitude reference + thrust: [roll, pitch, yaw, thrust]
class AttitudeReference : public InterBlockData<4> {
public:
    AttitudeReference() = default;

    double roll()  const { return data_[0]; }
    double pitch() const { return data_[1]; }
    double yaw()   const { return data_[2]; }

    Vec3 attitude() const { return Vec3(data_[0], data_[1], data_[2]); }

    void set_roll(double v)  { data_[0] = v; }
    void set_pitch(double v) { data_[1] = v; }
    void set_yaw(double v)   { data_[2] = v; }

    double thrust() const { return data_[3]; }
    void set_thrust(double v) { data_[3] = v; }
};

// External torque in body frame [Tx, Ty, Tz] in N·m.
class BodyTorque : public InterBlockData<3> {
public:
    BodyTorque() = default;
    explicit BodyTorque(const Vec3& t) { data_[0]=t.x(); data_[1]=t.y(); data_[2]=t.z(); }

    Vec3 torque() const { return Vec3(data_[0], data_[1], data_[2]); }
    void set_torque(const Vec3& t) { data_[0]=t.x(); data_[1]=t.y(); data_[2]=t.z(); }
};

// External force vector in NED frame [Fx, Fy, Fz] in Newtons.
class NedForce : public InterBlockData<3> {
public:
    NedForce() = default;
    explicit NedForce(const Vec3& f) { data_[0]=f.x(); data_[1]=f.y(); data_[2]=f.z(); }

    Vec3 force() const { return Vec3(data_[0], data_[1], data_[2]); }
    void set_force(const Vec3& f) { data_[0]=f.x(); data_[1]=f.y(); data_[2]=f.z(); }
};

// Empty signal for blocks with no input.
class NoInput : public InterBlockData<0> {
public:
    NoInput() = default;
};

} // namespace shared
