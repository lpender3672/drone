#pragma once

#include "../blocks/controller.hpp"
#include "../blocks/pid.hpp"
#include "../../shared/types/state.h"
#include "state.hpp"
#include <array>
#include <memory>

namespace sim {
namespace quadcopter {

/**
 * Cascaded attitude controller using scalar PID blocks.
 * Outer loop: P-only attitude -> rate setpoint
 * Inner loop: PID rate -> control output
 * Mixer: [thrust, roll, pitch, yaw] -> motor efforts
 */
class AttitudePidController : public ControllerBlock<shared::ObservedState, AttitudeReference, MotorEfforts> {
public:
    struct Params {
        // Attitude (outer) loop - P only
        Vec3 kp_attitude = Vec3(4.0, 4.0, 2.0);
        double max_rate_setpoint = 5.0;

        // Rate (inner) loop PID params
        std::array<PidParams, 3> rate_pid = {{
            {0.1, 0.0, 0.002, -0.3, 0.3, -1.0, 1.0},  // roll
            {0.1, 0.0, 0.002, -0.3, 0.3, -1.0, 1.0},  // pitch
            {0.15, 0.0, 0.0, -0.3, 0.3, -1.0, 1.0}    // yaw
        }};
    };

    static Eigen::Matrix<double, 4, 4> default_mixer() {
        Eigen::Matrix<double, 4, 4> M;
        M << 1.0,  1.0, -1.0,  1.0,
             1.0, -1.0,  1.0,  1.0,
             1.0, -1.0, -1.0, -1.0,
             1.0,  1.0,  1.0, -1.0;
        return M;
    }

    AttitudePidController(const std::string& name, double update_rate_hz = 1000.0)
        : ControllerBlock<shared::ObservedState, AttitudeReference, MotorEfforts>(
            name, nullptr, update_rate_hz)
        , mixer_(default_mixer())
    {
        for (int i = 0; i < 3; ++i) {
            rate_pids_[i] = std::make_unique<PidBlock>(
                name + "_rate_pid_" + std::to_string(i),
                update_rate_hz
            );
        }
    }

    void set_params(const Params& params) {
        params_ = params;
        for (int i = 0; i < 3; ++i) {
            rate_pids_[i]->set_params(params_.rate_pid[i]);
        }
    }

    Params& params() { return params_; }
    const Params& params() const { return params_; }

    void set_mixer(const Eigen::Matrix<double, 4, 4>& mixer) { mixer_ = mixer; }

    void reset() {
        for (auto& pid : rate_pids_) {
            pid->reset();
        }
    }

    bool update(double current_time_s) override {
        if (!is_due(current_time_s)) return false;
        mark_updated(current_time_s);

        const auto& in = input_.value;
        Vec3 euler = in.state.euler_angles();

        // Attitude P loop -> rate setpoints
        Vec3 attitude_error = in.reference.attitude() - euler;
        attitude_error.z() = wrap_angle(attitude_error.z());

        Vec3 rate_setpoint = params_.kp_attitude.cwiseProduct(attitude_error);
        rate_setpoint = rate_setpoint.cwiseMax(-params_.max_rate_setpoint)
                                    .cwiseMin(params_.max_rate_setpoint);

        // Rate PID loops
        Vec3 rate_output;
        for (int i = 0; i < 3; ++i) {
            PidInput pid_in;
            pid_in.set_setpoint(rate_setpoint[i]);
            pid_in.set_measurement(in.state.angular_velocity[i]);

            rate_pids_[i]->input().set(pid_in);
            rate_pids_[i]->update(current_time_s);
            rate_output[i] = rate_pids_[i]->output().get().value();
        }

        // Mix to motor efforts
        Vec4 control_input(in.reference.thrust(), rate_output.x(), rate_output.y(), rate_output.z());
        output_.value.vector() = mixer_ * control_input;
        //output_.value.clamp();
        //output_.value.set_timestamp(current_time_s);

        return true;
    }

private:
    static double wrap_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    Params params_;
    Eigen::Matrix<double, 4, 4> mixer_;
    std::array<std::unique_ptr<PidBlock>, 3> rate_pids_;
};

} // namespace quadcopter
} // namespace sim