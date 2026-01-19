#pragma once

#include "../blocks/dynamics.hpp"
#include "motor.hpp"
#include "../../shared/types/state.h"
#include "../data/state.hpp"
#include "state.hpp"

namespace sim {
namespace quadcopter {

class QuadrotorDynamics : public DynamicsBlock<MotorEfforts, shared::TrueState> {
public:
    struct Params {
        double mass = 0.5;
        double arm_length = 0.175;
        Vec3 inertia = Vec3(0.0035, 0.0035, 0.0055);
        double drag_coeff = 0.1;
        double gravity = 9.81;
        
        LinearFirstOrderMotor::Params motor;
        PropellerParams propeller;
    };

    QuadrotorDynamics(const std::string& name,
                      double update_rate_hz = 1000.0,
                      double internal_rate_hz = 10000.0)
        : DynamicsBlock<MotorEfforts, shared::TrueState>(name, update_rate_hz, internal_rate_hz)
        , internal_rate_hz_(internal_rate_hz) {}

    void set_params(const Params& params) {
        params_ = params;
        
        for (int i = 0; i < 4; ++i) {
            motors_[i] = std::make_unique<LinearFirstOrderMotor>(
                name_ + "_motor" + std::to_string(i),
                params_.motor,
                params_.propeller,
                internal_rate_hz_
            );
        }
    }
    
    Params& params() { return params_; }

    void reset(const shared::TrueState& initial_state) override {
        DynamicsBlock::reset(initial_state);
    }

protected:
    void step_dynamics(double dt) override {
        for (int i = 0; i < 4; ++i) {
            motors_[i]->input().set(this->input_.value[i]);
            motors_[i]->update(this->last_update_time_s_);
        }

        double thrust[4], torque[4];
        for (int i = 0; i < 4; ++i) {
            thrust[i] = motors_[i]->output().get().thrust();
            torque[i] = motors_[i]->output().get().torque();
        }

        Mat3 R_nb = this->output_.value.R_nb();

        double total_thrust = thrust[0] + thrust[1] + thrust[2] + thrust[3];
        Vec3 thrust_ned = R_nb * Vec3(0.0, 0.0, -total_thrust);

        Vec3 drag = -params_.drag_coeff * this->output_.value.velocity;
        Vec3 gravity_vec(0.0, 0.0, params_.mass * params_.gravity);

        Vec3 accel = (thrust_ned + drag + gravity_vec) / params_.mass;
        this->output_.value.velocity += accel * dt;
        this->output_.value.position += this->output_.value.velocity * dt;

        if (this->output_.value.position.z() > 0.0 && this->output_.value.velocity.z() > 0.0) {
            this->output_.value.position.z() = 0.0;
            this->output_.value.velocity.z() = 0.0;
        }

        double L = params_.arm_length;
        double tau_roll = L * (thrust[0] + thrust[3] - thrust[1] - thrust[2]);
        double tau_pitch = L * (thrust[1] + thrust[3] - thrust[0] - thrust[2]);
        double tau_yaw = torque[0] + torque[1] - torque[2] - torque[3];

        Vec3 body_torque(tau_roll, tau_pitch, tau_yaw);

        Vec3 omega = this->output_.value.angular_velocity;
        Vec3 I = params_.inertia;
        
        Vec3 omega_dot;
        omega_dot.x() = (body_torque.x() - (I.z() - I.y()) * omega.y() * omega.z()) / I.x();
        omega_dot.y() = (body_torque.y() - (I.x() - I.z()) * omega.x() * omega.z()) / I.y();
        omega_dot.z() = (body_torque.z() - (I.y() - I.x()) * omega.x() * omega.y()) / I.z();

        this->output_.value.angular_velocity += omega_dot * dt;

        Quat omega_quat(0.0, omega.x(), omega.y(), omega.z());
        Quat q_dot;
        q_dot.coeffs() = 0.5 * (this->output_.value.attitude * omega_quat).coeffs();
        this->output_.value.attitude.coeffs() += q_dot.coeffs() * dt;
        this->output_.value.normalise_quaternion();
    }

private:
    Params params_;
    double internal_rate_hz_;
    std::array<std::unique_ptr<Motor>, 4> motors_;
};

} // namespace quadcopter
} // namespace sim