#pragma once

#include "../core/block.hpp"
#include "../../shared/sensors/sensor_base.h"
#include <deque>
#include <random>
#include <optional>

namespace sim {

template<typename TrueStateT, typename ReadingT>
class SensorBlock : public TypedBlock<TrueStateT, ReadingT>, 
                    public sensors::Sensor<ReadingT> {
public:
    SensorBlock(const std::string& name, 
                double update_rate_hz, 
                double latency_s = 0.0)
        : TypedBlock<TrueStateT, ReadingT>(name, "true_state", "reading", update_rate_hz)
        , sensors::Sensor<ReadingT>(name.c_str(), static_cast<uint32_t>(1e6 / update_rate_hz))
        , latency_s_(latency_s)
        , rng_(std::random_device{}())
    {
        this->set_initialized(true);
    }

    std::optional<ReadingT> get_reading() const override {
        if (latency_buffer_.empty()) {
            return std::nullopt;
        }
        
        double current_time = latency_buffer_.back().timestamp();
        for (const auto& reading : latency_buffer_) {
            if (current_time - reading.timestamp() >= latency_s_ - 1e-9) {
                return reading;
            }
        }
        return std::nullopt;
    }

    bool has_new_reading() const { return new_reading_available_; }
    void clear_new_reading_flag() { new_reading_available_ = false; }

    bool update(double current_time_s) override {
        if (!this->is_due(current_time_s)) return false;
        
        this->mark_updated(current_time_s);

        const auto& true_state = this->input_.value;
        
        ReadingT reading = sample(true_state, current_time_s);
        reading.set_timestamp(current_time_s);
        
        latency_buffer_.push_back(reading);
        
        while (latency_buffer_.size() > 1) {
            double age = current_time_s - latency_buffer_.front().timestamp();
            if (age > latency_s_ + 0.1) {
                latency_buffer_.pop_front();
            } else {
                break;
            }
        }
        
        new_reading_available_ = true;
        
        auto delayed = get_reading();
        if (delayed) {
            this->output_.value = *delayed;
            this->notify_output(*delayed);
        }
        
        return true;
    }

    // Embedded interface (not used in sim)
    void update(uint32_t current_time_us) override {
        update(current_time_us / 1e6);
    }

    bool initialize() override { return true; }

    void set_latency(double latency_s) { latency_s_ = latency_s; }
    double latency() const { return latency_s_; }

protected:
    virtual ReadingT sample(const TrueStateT& true_state, double current_time_s) = 0;

    double gaussian_noise(double stddev) {
        return std::normal_distribution<double>(0.0, stddev)(rng_);
    }

    double latency_s_;
    std::deque<ReadingT> latency_buffer_;
    bool new_reading_available_ = false;
    std::mt19937 rng_;
};

} // namespace sim