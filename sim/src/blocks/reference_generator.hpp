#pragma once

#include "../core/block.hpp"
#include <functional>
#include <cmath>

namespace sim {

/**
 * Generates scalar reference signals for testing.
 */
class ScalarReferenceGenerator : public TypedBlock<NoInput, Scalar> {
public:
    enum class SignalType {
        CONSTANT,
        STEP,
        RAMP,
        SINUSOID,
        SQUARE,
        CUSTOM
    };

    ScalarReferenceGenerator(const std::string& name, uint32_t update_period_us = 0.0)
        : TypedBlock(name, "none", "reference", update_period_us)
    {}

    void set_constant(double value) {
        signal_type_ = SignalType::CONSTANT;
        constant_value_ = value;
    }

    void set_step(double step_time, double before, double after) {
        signal_type_ = SignalType::STEP;
        step_time_ = step_time;
        step_before_ = before;
        step_after_ = after;
    }

    void set_ramp(double start_time, double start_value, double slope) {
        signal_type_ = SignalType::RAMP;
        ramp_start_time_ = start_time;
        ramp_start_value_ = start_value;
        ramp_slope_ = slope;
    }

    void set_sinusoid(double amplitude, double frequency_hz, 
                      double offset = 0.0, double phase_rad = 0.0) {
        signal_type_ = SignalType::SINUSOID;
        sin_amplitude_ = amplitude;
        sin_frequency_ = frequency_hz;
        sin_offset_ = offset;
        sin_phase_ = phase_rad;
    }

    void set_square(double amplitude, double frequency_hz, double offset = 0.0) {
        signal_type_ = SignalType::SQUARE;
        square_amplitude_ = amplitude;
        square_frequency_ = frequency_hz;
        square_offset_ = offset;
    }

    void set_custom(std::function<double(double)> fn) {
        signal_type_ = SignalType::CUSTOM;
        custom_fn_ = std::move(fn);
    }

    bool update(uint64_t current_time_us) override {
        if (!is_due(current_time_us)) return false;
        mark_updated(current_time_us);

        double current_time_s = current_time_us / 1e6;
        double value = 0.0;

        switch (signal_type_) {
            case SignalType::CONSTANT:
                value = constant_value_;
                break;

            case SignalType::STEP:
                value = (current_time_s < step_time_) ? step_before_ : step_after_;
                break;

            case SignalType::RAMP:
                if (current_time_s < ramp_start_time_) {
                    value = ramp_start_value_;
                } else {
                    value = ramp_start_value_ + ramp_slope_ * (current_time_s - ramp_start_time_);
                }
                break;

            case SignalType::SINUSOID: {
                double phase = 2.0 * M_PI * sin_frequency_ * current_time_s + sin_phase_;
                value = sin_offset_ + sin_amplitude_ * std::sin(phase);
                break;
            }

            case SignalType::SQUARE: {
                double period = 1.0 / square_frequency_;
                double t_mod = std::fmod(current_time_s, period);
                value = square_offset_ + ((t_mod < period / 2.0) ? square_amplitude_ : -square_amplitude_);
                break;
            }

            case SignalType::CUSTOM:
                if (custom_fn_) {
                    value = custom_fn_(current_time_s);
                }
                break;
        }

        output_.value = Scalar(value, current_time_us);
        return true;
    }

private:
    SignalType signal_type_ = SignalType::CONSTANT;

    double constant_value_ = 0.0;

    double step_time_ = 0.0;
    double step_before_ = 0.0;
    double step_after_ = 0.0;

    double ramp_start_time_ = 0.0;
    double ramp_start_value_ = 0.0;
    double ramp_slope_ = 0.0;

    double sin_amplitude_ = 0.0;
    double sin_frequency_ = 0.0;
    double sin_offset_ = 0.0;
    double sin_phase_ = 0.0;

    double square_amplitude_ = 0.0;
    double square_frequency_ = 0.0;
    double square_offset_ = 0.0;

    std::function<double(double)> custom_fn_;
};

} // namespace sim