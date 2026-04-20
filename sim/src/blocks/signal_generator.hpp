#pragma once
#include "../core/block.hpp"
#include "../data/state.hpp"
#include <cmath>

namespace sim {

enum class Waveform { Impulse, Step, Ramp, Sine };

struct WaveformParams {
    Waveform type         = Waveform::Step;
    double   amplitude    = 1.0;
    double   start_time_s = 0.0;
    double   duration_s   = 0.01;   // impulse: pulse width; ramp: ramp-up time
    double   frequency_hz = 1.0;   // sine only
    double   bias         = 0.0;
};

class WaveformGenerator : public SourceBlock<Scalar> {
public:
    WaveformGenerator(const std::string& name,
                      const WaveformParams& params,
                      uint32_t update_period_us = 0)
        : SourceBlock<Scalar>(name, "signal", update_period_us)
        , params_(params)
    {}

    void set_params(const WaveformParams& p) { params_ = p; }
    const WaveformParams& params() const { return params_; }

    double value() const { return output_.value.value(); }

    bool update(uint64_t current_time_us) override {
        if (!is_due(current_time_us)) return false;

        const double t  = static_cast<double>(current_time_us) * 1e-6;
        const double dt = t - params_.start_time_s;
        double val = params_.bias;

        if (dt >= 0.0) {
            switch (params_.type) {
            case Waveform::Impulse:
                if (dt < params_.duration_s) val += params_.amplitude;
                break;
            case Waveform::Step:
                val += params_.amplitude;
                break;
            case Waveform::Ramp:
                val += params_.amplitude * std::min(dt / params_.duration_s, 1.0);
                break;
            case Waveform::Sine:
                val += params_.amplitude * std::sin(2.0 * M_PI * params_.frequency_hz * dt);
                break;
            }
        }

        output_.value.set_value(val);
        mark_updated(current_time_us);
        return true;
    }

private:
    WaveformParams params_;
};

} // namespace sim
