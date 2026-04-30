#pragma once

#include "../blocks/dynamics.hpp"
#include "motor.hpp"
#include "../../shared/types/state.h"
#include "../data/state.hpp"
#include "state.hpp"

namespace sim {
namespace quadcopter {

class QuadrotorDynamics : public DynamicsBlock<MotorEfforts, TrueState> {
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
                      const Params& params,
                      uint32_t update_period_us = 1000u,
                      uint32_t internal_period_us = 10000u)
        : DynamicsBlock<MotorEfforts, TrueState>(name, update_period_us, internal_period_us)
        , params_(params)
        , internal_period_us_(internal_period_us)
        , motor_throttles_{{
              OutputPort<Scalar>(name + "_throttle_0"),
              OutputPort<Scalar>(name + "_throttle_1"),
              OutputPort<Scalar>(name + "_throttle_2"),
              OutputPort<Scalar>(name + "_throttle_3"),
          }}
    {
        for (int i = 0; i < 4; ++i) {
            motors_[i] = std::make_unique<LinearFirstOrderMotor>(
                name_ + "_motor" + std::to_string(i),
                params_.motor,
                params_.propeller,
                internal_period_us_
            );
            // Wire the parent's per-motor throttle output to the child's
            // input once. step_dynamics() then writes the per-axis throttle
            // and the motor reads through the connection.
            connect(motor_throttles_[i], motors_[i]->input());
        }
    }

    const Params& params() const { return params_; }

    void reset(const TrueState& initial_state) override {
        DynamicsBlock::reset(initial_state);
        // Initialize motors at hover RPM to avoid spin-up transient
        double hover_omega = std::sqrt(params_.mass * params_.gravity /
            (4.0 * params_.propeller.k_t * params_.propeller.rho *
             std::pow(params_.propeller.d, 4)));
        double hover_thrust = params_.propeller.k_t * params_.propeller.rho *
            hover_omega * hover_omega * std::pow(params_.propeller.d, 4);
        double hover_torque = params_.propeller.k_q * params_.propeller.rho *
            hover_omega * hover_omega * std::pow(params_.propeller.d, 5);
        for (int i = 0; i < 4; ++i) {
            motors_[i]->output().value.set_omega(hover_omega);
            motors_[i]->output().value.set_thrust(hover_thrust);
            motors_[i]->output().value.set_torque(hover_torque);
        }
    }

    InputPort<NedForce>&   disturbance_input()        { return disturbance_input_; }
    InputPort<BodyTorque>& disturbance_torque_input() { return disturbance_torque_input_; }

protected:
    void step_dynamics(double dt) override {
        // Throttles don't change within an outer tick — write once, then
        // sub-step the motors. They read back through the connect() wiring.
        for (int i = 0; i < 4; ++i) {
            motor_throttles_[i].set(Scalar(this->input_.get()[i]));
            motors_[i]->update(this->last_update_time_us_);
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

        Vec3 ext_force = disturbance_input_.connected ? disturbance_input_.get().force() : Vec3::Zero();
        Vec3 accel = (thrust_ned + drag + gravity_vec + ext_force) / params_.mass;
        this->output_.value.linear_accel = accel;
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

        Vec3 ext_torque = disturbance_torque_input_.connected
            ? disturbance_torque_input_.get().torque() : Vec3::Zero();
        Vec3 body_torque = Vec3(tau_roll, tau_pitch, tau_yaw) + ext_torque;

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
    uint32_t internal_period_us_;
    std::array<std::unique_ptr<Motor>, 4> motors_;
    std::array<OutputPort<Scalar>, 4>     motor_throttles_;
    InputPort<NedForce>   disturbance_input_{"disturbance"};
    InputPort<BodyTorque> disturbance_torque_input_{"disturbance_torque"};
};

} // namespace quadcopter
} // namespace sim