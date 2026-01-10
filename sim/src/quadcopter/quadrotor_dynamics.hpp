#pragma once

#include "../blocks/dynamics_model.hpp"
#include "motor_efforts.hpp"

namespace sim {
namespace quadcopter {

/**
 * Rigid body quadrotor dynamics model.
 * 
 * Assumptions:
 * - X configuration
 * - Symmetric mass distribution
 * - Linear thrust model (thrust = k * omega^2, but we work with normalised inputs)
 * - Simple linear drag
 */
class QuadrotorDynamics : public DynamicsModel<4> {
public:
    struct Params {
        // Physical properties
        double mass = 0.5;           // [kg]
        double arm_length = 0.175;   // [m] distance from CoG to motor
        
        // Inertia tensor (diagonal, symmetric quad)
        Vec3 inertia = Vec3(0.0035, 0.0035, 0.0055);  // [Ixx, Iyy, Izz] [kg·m²]
        
        // Motor properties
        double max_thrust = 4.0;     // [N] per motor at full throttle
        double torque_coeff = 0.01;  // reaction torque coefficient (tau = k * thrust)
        double motor_time_const = 0.02; // [s] first-order motor lag
        
        // Aerodynamics
        double drag_coeff = 0.1;     // linear drag coefficient
        
        // Environment
        double gravity = 9.81;       // [m/s²]
    };

    QuadrotorDynamics(const std::string& name = "quadrotor_dynamics", 
                      double update_rate_hz = 1000.0,
                      double internal_rate_hz = 10000.0)
        : DynamicsModel<4>(name, update_rate_hz, internal_rate_hz)
    {}

    void set_params(const Params& params) { params_ = params; }
    Params& params() { return params_; }

    void reset(const State& initial_state) override {
        DynamicsModel::reset(initial_state);
        motor_thrust_ = Vec4::Zero();
    }

    // Convenience method to set motor efforts
    void set_motor_efforts(const MotorEfforts& efforts) {
        set_control_efforts(efforts);
    }

protected:
    void step_dynamics(double dt) override {
        // Update motor thrusts with first-order lag
        Vec4 target_thrust = control_efforts_.efforts * params_.max_thrust;
        double alpha = dt / (params_.motor_time_const + dt);
        motor_thrust_ += alpha * (target_thrust - motor_thrust_);

        // Rotation matrix from body to NED
        Mat3 R_nb = state_.R_nb();

        // Total thrust (sum of motors, pointing up in body frame = -z body)
        double total_thrust = motor_thrust_.sum();

        // Thrust vector in body frame is [0, 0, -total_thrust]
        // Transform to NED frame
        Vec3 thrust_body(0.0, 0.0, -total_thrust);
        Vec3 thrust_ned = R_nb * thrust_body;

        // Linear drag in NED (simple model)
        Vec3 drag = -params_.drag_coeff * state_.velocity;

        // Gravity in NED
        Vec3 gravity_vec(0.0, 0.0, params_.mass * params_.gravity);

        // Translational dynamics (NED frame)
        Vec3 accel = (thrust_ned + drag) / params_.mass + gravity_vec / params_.mass;

        // Update velocity and position
        state_.velocity += accel * dt;
        state_.position += state_.velocity * dt;

        // Ground constraint (crude)
        if (state_.position.z() > 0.0 && state_.velocity.z() > 0.0) {
            state_.position.z() = 0.0;
            state_.velocity.z() = 0.0;
        }

        // Torques in body frame (X configuration)
        // Motor layout:
        //   2(CCW)   0(CW)
        //       \   /
        //         X
        //       /   \
        //   1(CW)   3(CCW)
        double L = params_.arm_length;
        double k_torque = params_.torque_coeff;

        // Roll torque: right motors (+) - left motors (-)
        double tau_roll = L * (motor_thrust_(0) + motor_thrust_(3) - 
                              motor_thrust_(1) - motor_thrust_(2));

        // Pitch torque: front motors (-) - back motors (+) 
        double tau_pitch = L * (motor_thrust_(1) + motor_thrust_(3) - 
                               motor_thrust_(0) - motor_thrust_(2));

        // Yaw torque: CW motors (+) - CCW motors (-)
        double tau_yaw = k_torque * (motor_thrust_(0) + motor_thrust_(1) - 
                                     motor_thrust_(2) - motor_thrust_(3));

        Vec3 torque(tau_roll, tau_pitch, tau_yaw);

        // Angular acceleration (Euler's equations)
        // I * omega_dot = torque - omega x (I * omega)
        Vec3 omega = state_.angular_velocity;
        Vec3 I = params_.inertia;
        Vec3 I_omega(I.x() * omega.x(), I.y() * omega.y(), I.z() * omega.z());
        
        Vec3 omega_dot;
        omega_dot.x() = (torque.x() - (I.z() - I.y()) * omega.y() * omega.z()) / I.x();
        omega_dot.y() = (torque.y() - (I.x() - I.z()) * omega.x() * omega.z()) / I.y();
        omega_dot.z() = (torque.z() - (I.y() - I.x()) * omega.x() * omega.y()) / I.z();

        // Update angular velocity
        state_.angular_velocity += omega_dot * dt;

        // Quaternion derivative: q_dot = 0.5 * q ⊗ [0, omega]
        Quat omega_quat(0.0, omega.x(), omega.y(), omega.z());
        Quat q_dot;
        q_dot.coeffs() = 0.5 * (state_.attitude * omega_quat).coeffs();

        // Update quaternion
        state_.attitude.coeffs() += q_dot.coeffs() * dt;
        state_.normalise_quaternion();
    }

private:
    Params params_;
    Vec4 motor_thrust_ = Vec4::Zero();  // Current thrust per motor [N]
};

} // namespace quadcopter
} // namespace sim
