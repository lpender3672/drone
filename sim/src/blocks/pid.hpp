#pragma once

#include "../core/block.hpp"
#include "../data/state.hpp"
#include <algorithm>
#include <cmath>

namespace sim {

struct PidParams {
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;
    double output_min = -1.0;
    double output_max = 1.0;
    double integral_min = -1.0;
    double integral_max = 1.0;
};


class PidBlock : public TypedBlock<PidInput, Scalar> {
public:
    explicit PidBlock(const std::string& name, double update_rate_hz = 0.0)
        : TypedBlock(name, "input", "output", update_rate_hz) {}

    void set_params(const PidParams& params) { params_ = params; }
    PidParams& params() { return params_; }
    const PidParams& params() const { return params_; }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        output_.value = Scalar();
        first_update_ = true;
    }

    bool update(double current_time_s) override {
        if (!is_due(current_time_s)) return false;

        double dt = update_period_s_;
        if (!first_update_ && last_update_time_s_ > -1e8) {
            dt = current_time_s - last_update_time_s_;
        }

        mark_updated(current_time_s);

        const auto& in = input_.value;
        double error = in.setpoint() - in.measurement();

        double p = params_.kp * error;

        if (!first_update_) {
            integral_ += error * dt;
            integral_ = std::clamp(integral_, params_.integral_min, params_.integral_max);
        }
        double i = params_.ki * integral_;

        double d = 0.0;
        if (!first_update_ && dt > 0.0) {
            d = params_.kd * (error - prev_error_) / dt;
        }
        prev_error_ = error;

        first_update_ = false;

        double output = std::clamp(p + i + d, params_.output_min, params_.output_max);
        output_.value = Scalar(output, current_time_s);

        return true;
    }

private:
    PidParams params_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool first_update_ = true;
};

} // namespace sim