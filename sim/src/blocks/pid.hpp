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
    explicit PidBlock(const std::string& name, uint32_t update_period_us = 0.0)
        : TypedBlock(name, "input", "output", update_period_us) {}

    void set_params(const PidParams& params) { params_ = params; }
    PidParams& params() { return params_; }
    const PidParams& params() const { return params_; }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        output_.value = Scalar();
        has_updated_ = false;
    }

    bool update(uint64_t current_time_us) override {
        if (!is_due(current_time_us)) return false;

        double dt = get_dt_us(current_time_us) / 1e6;

        mark_updated(current_time_us);

        const auto& in = input_.value;
        double error = in.setpoint() - in.measurement();

        double p = params_.kp * error;

        if (has_updated_) {
            integral_ += error * dt;
            integral_ = std::clamp(integral_, params_.integral_min, params_.integral_max);
        }
        double i = params_.ki * integral_;

        double d = 0.0;
        if (has_updated_ && dt > 0.0) {
            d = params_.kd * (error - prev_error_) / dt;
        }
        prev_error_ = error;

        double output = std::clamp(p + i + d, params_.output_min, params_.output_max);
        output_.value = Scalar(output, current_time_us);

        return true;
    }

private:
    PidParams params_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
};

} // namespace sim