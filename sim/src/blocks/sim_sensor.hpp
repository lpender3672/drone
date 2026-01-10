#pragma once

#include "../core/block.hpp"
#include "../data/state.hpp"
#include "../data/sensor_reading.hpp"
#include "../../../shared/sensors/sensor_base.h"
#include <deque>
#include <memory>
#include <random>
#include <optional>

namespace sim {

/**
 * Simulation sensor that extends the shared sensor base with simulation-specific features.
 * Adds latency simulation and noise generation capabilities.
 * 
 * Inherits timing logic from Sensor.
 * Inherits block management from Block.
 * 
 * Derived classes implement the actual sensor model (what gets measured, how).
 */
template<typename ReadingType>
class SimSensor : public Block, public sensors::Sensor<ReadingType> {
public:
    SimSensor(const std::string& name, double update_rate_hz, double latency_s = 0.0)
        : Block(name, update_rate_hz)
        , sensors::Sensor<ReadingType>(name.c_str(), static_cast<uint32_t>(1e6 / update_rate_hz))
        , latency_s_(latency_s)
        , rng_(std::random_device{}())
    {
        this->set_initialized(true);
    }

    // Set the true state pointer (called by simulation manager)
    void set_true_state_source(const State* state) {
        true_state_ = state;
    }

    // Get latest reading (after latency) - implements Sensor interface
    std::optional<ReadingType> get_reading() const override {
        if (latency_buffer_.empty()) {
            return std::nullopt;
        }
        
        // Find reading that's old enough
        double current_time = latency_buffer_.back().timestamp();
        for (const auto& reading : latency_buffer_) {
            if (current_time - reading.timestamp() >= latency_s_ - 1e-9) {
                return reading;
            }
        }
        return std::nullopt;
    }

    // Check if new reading is available since last call
    bool has_new_reading() const { return new_reading_available_; }
    void clear_new_reading_flag() { new_reading_available_ = false; }

    // Block interface
    bool update(double current_time_s) override {
        if (!is_due(current_time_s)) return false;
        if (!true_state_) return false;

        // Sample the sensor
        ReadingType reading = sample(*true_state_, current_time_s);
        reading.set_timestamp(current_time_s);
        
        // Add to latency buffer
        latency_buffer_.push_back(reading);
        
        // Trim old readings from buffer
        while (latency_buffer_.size() > 1) {
            double age = current_time_s - latency_buffer_.front().timestamp();
            if (age > latency_s_ + 0.1) {  // Keep a bit of margin
                latency_buffer_.pop_front();
            } else {
                break;
            }
        }
        
        new_reading_available_ = true;
        mark_updated(current_time_s);
        
        // Notify with the delayed reading if available
        auto delayed = get_reading();
        if (delayed) {
            notify_output(*delayed);
        }
        
        return true;
    }

    // Sensor interface (not used in sim, but required by base)
    void update(uint32_t current_time_us) override {
        // Convert to seconds and call the main update
        update(current_time_us / 1e6);
    }

    bool initialize() override {
        // Simulation sensors are always initialized
        return true;
    }

    // Configuration
    void set_latency(double latency_s) { latency_s_ = latency_s; }
    double latency() const { return latency_s_; }

protected:
    // Override this to implement sensor model
    // Should read from true_state and add appropriate noise
    virtual ReadingType sample(const State& true_state, double current_time_s) = 0;

    // Noise generation helpers
    double gaussian_noise(double stddev) {
        return std::normal_distribution<double>(0.0, stddev)(rng_);
    }

    const State* true_state_ = nullptr;
    double latency_s_;
    
    std::deque<ReadingType> latency_buffer_;
    bool new_reading_available_ = false;
    
    std::mt19937 rng_;
};

} // namespace sim
