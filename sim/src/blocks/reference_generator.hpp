#pragma once

#include "../core/block.hpp"
#include "../data/reference.hpp"
#include <functional>

namespace sim {

/**
 * Generates reference/setpoint signals for testing.
 * Can produce step, ramp, sinusoidal, or custom signals.
 */
class ReferenceGenerator : public Block {
public:
    enum class SignalType {
        CONSTANT,
        STEP,
        RAMP,
        SINUSOID,
        CUSTOM
    };

    ReferenceGenerator(const std::string& name = "reference_generator")
        : Block(name, 0.0)  // Updates every call
    {}

    // Get current reference
    const Reference& reference() const { return reference_; }

    // Set constant reference
    void set_constant(const Reference& ref) {
        signal_type_ = SignalType::CONSTANT;
        reference_ = ref;
    }

    // Set step input
    void set_step(double step_time, const Reference& before, const Reference& after) {
        signal_type_ = SignalType::STEP;
        step_time_ = step_time;
        ref_before_ = before;
        ref_after_ = after;
    }

    // Set sinusoidal roll/pitch/yaw with given amplitude and frequency
    void set_sinusoid(double amplitude_rad, double frequency_hz, 
                      bool roll = true, bool pitch = false, bool yaw = false,
                      double base_thrust = 0.5) {
        signal_type_ = SignalType::SINUSOID;
        sin_amplitude_ = amplitude_rad;
        sin_frequency_ = frequency_hz;
        sin_axes_ = Vec3(roll ? 1.0 : 0.0, pitch ? 1.0 : 0.0, yaw ? 1.0 : 0.0);
        reference_.thrust = base_thrust;
    }

    // Set custom function: f(time) -> Reference
    void set_custom(std::function<Reference(double)> fn) {
        signal_type_ = SignalType::CUSTOM;
        custom_fn_ = std::move(fn);
    }

    bool update(double current_time_s) override {
        switch (signal_type_) {
            case SignalType::CONSTANT:
                // reference_ already set
                break;

            case SignalType::STEP:
                reference_ = (current_time_s < step_time_) ? ref_before_ : ref_after_;
                break;

            case SignalType::RAMP:
                // TODO: implement ramp
                break;

            case SignalType::SINUSOID: {
                double phase = 2.0 * M_PI * sin_frequency_ * current_time_s;
                double value = sin_amplitude_ * std::sin(phase);
                reference_.attitude = sin_axes_ * value;
                break;
            }

            case SignalType::CUSTOM:
                if (custom_fn_) {
                    reference_ = custom_fn_(current_time_s);
                }
                break;
        }

        reference_.set_timestamp(current_time_s);
        mark_updated(current_time_s);
        notify_output(reference_);

        return true;
    }

private:
    SignalType signal_type_ = SignalType::CONSTANT;
    Reference reference_;

    // Step parameters
    double step_time_ = 0.0;
    Reference ref_before_;
    Reference ref_after_;

    // Sinusoid parameters
    double sin_amplitude_ = 0.0;
    double sin_frequency_ = 0.0;
    Vec3 sin_axes_ = Vec3::Zero();

    // Custom function
    std::function<Reference(double)> custom_fn_;
};

} // namespace sim
