#pragma once

#include "../blocks/controller.hpp"
#include "motor_efforts.hpp"

namespace sim {
namespace quadcopter {

/**
 * Simple PID controller for attitude + thrust.
 * Implements cascaded rate + attitude loops for roll/pitch/yaw.
 */
class PidController : public Controller<4> {
public:
    struct Gains {
        // Attitude (outer) loop - P only
        Vec3 kp_attitude = Vec3(4.0, 4.0, 2.0);  // roll, pitch, yaw

        // Rate (inner) loop - PID
        Vec3 kp_rate = Vec3(0.1, 0.1, 0.15);
        Vec3 ki_rate = Vec3(0.0, 0.0, 0.0);
        Vec3 kd_rate = Vec3(0.002, 0.002, 0.0);

        // Output limits
        double max_rate_setpoint = 5.0;  // rad/s
        double max_output = 0.3;          // max correction per axis
    };

    // Mixer matrix: converts [thrust, roll, pitch, yaw] to motor efforts
    // Each row is a motor, columns are [thrust, roll, pitch, yaw]
    // Default X-configuration mixer
    static Eigen::Matrix<double, 4, 4> default_mixer() {
        Eigen::Matrix<double, 4, 4> M;
        // Motor 0: front-right (CW)  - +roll, -pitch, +yaw
        // Motor 1: back-left (CW)    - -roll, +pitch, +yaw
        // Motor 2: front-left (CCW)  - -roll, -pitch, -yaw
        // Motor 3: back-right (CCW)  - +roll, +pitch, -yaw
        M << 1.0,  1.0, -1.0,  1.0,   // motor 0
             1.0, -1.0,  1.0,  1.0,   // motor 1
             1.0, -1.0, -1.0, -1.0,   // motor 2
             1.0,  1.0,  1.0, -1.0;   // motor 3
        return M;
    }

    PidController(const std::string& name = "pid_controller", double update_rate_hz = 1000.0)
        : Controller<4>(name, update_rate_hz)
        , mixer_(default_mixer())
    {}

    void set_gains(const Gains& gains) { gains_ = gains; }
    Gains& gains() { return gains_; }

    void set_mixer(const Eigen::Matrix<double, 4, 4>& mixer) { mixer_ = mixer; }

    void reset() override {
        Controller<4>::reset();
        rate_integral_ = Vec3::Zero();
        last_rate_error_ = Vec3::Zero();
        first_update_ = true;
    }

    // Get output as MotorEfforts (convenience)
    MotorEfforts motor_efforts() const {
        return MotorEfforts(control_efforts_.efforts, control_efforts_.timestamp());
    }

    bool update(double current_time_s) override {
        if (!is_due(current_time_s)) return false;

        double dt = update_period_s_;
        if (!first_update_ && last_update_time_s_ > 0) {
            dt = current_time_s - last_update_time_s_;
        }
        first_update_ = false;

        // Get current Euler angles
        Vec3 euler = estimated_state_.euler_angles();

        // Attitude loop: angle error -> rate setpoint
        Vec3 attitude_error = reference_.attitude - euler;
        
        // Wrap yaw error to [-pi, pi]
        attitude_error.z() = wrap_angle(attitude_error.z());

        Vec3 rate_setpoint = gains_.kp_attitude.cwiseProduct(attitude_error);
        rate_setpoint = rate_setpoint.cwiseMax(-gains_.max_rate_setpoint)
                                     .cwiseMin(gains_.max_rate_setpoint);

        // Rate loop: rate error -> control output
        Vec3 rate_error = rate_setpoint - estimated_state_.angular_velocity;

        // PID on rate
        rate_integral_ += rate_error * dt;
        Vec3 rate_deriv = (rate_error - last_rate_error_) / dt;
        last_rate_error_ = rate_error;

        Vec3 rate_output = gains_.kp_rate.cwiseProduct(rate_error)
                         + gains_.ki_rate.cwiseProduct(rate_integral_)
                         + gains_.kd_rate.cwiseProduct(rate_deriv);

        // Clamp outputs
        rate_output = rate_output.cwiseMax(-gains_.max_output).cwiseMin(gains_.max_output);

        // Mix to motor outputs
        Vec4 control_input(reference_.thrust, rate_output.x(), rate_output.y(), rate_output.z());
        control_efforts_.efforts = mixer_ * control_input;
        control_efforts_.clamp();
        control_efforts_.set_timestamp(current_time_s);

        mark_updated(current_time_s);
        notify_output(control_efforts_);

        return true;
    }

private:
    static double wrap_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    Gains gains_;
    Eigen::Matrix<double, 4, 4> mixer_;

    // Integrator states
    Vec3 rate_integral_ = Vec3::Zero();
    Vec3 last_rate_error_ = Vec3::Zero();
    
    bool first_update_ = true;
};

} // namespace quadcopter
} // namespace sim
