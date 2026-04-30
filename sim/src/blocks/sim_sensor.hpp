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
                uint32_t update_period_us, 
                uint32_t latency_us = 0)
        : TypedBlock<TrueStateT, ReadingT>(name, "true_state", "reading", update_period_us)
        , sensors::Sensor<ReadingT>(name.c_str(), update_period_us)
        , latency_us_(latency_us)
        , rng_(std::random_device{}())
    {
        this->set_initialized(true);
    }

    std::optional<ReadingT> get_reading() const override {
        if (latency_buffer_.empty()) {
            return std::nullopt;
        }

        if (latency_us_ == 0) {
            return latency_buffer_.back();
        }

        uint64_t current_time = latency_buffer_.back().timestamp();
        // Search from most-recent to oldest: return the newest reading that is
        // at least latency_us_ old (so the output advances every step, not stuck).
        for (auto it = latency_buffer_.rbegin(); it != latency_buffer_.rend(); ++it) {
            if (current_time - it->timestamp() >= latency_us_) {
                return *it;
            }
        }
        return std::nullopt;
    }

    bool has_new_reading() const { return new_reading_available_; }
    void clear_new_reading_flag() { new_reading_available_ = false; }

    bool update(uint64_t current_time_us) override {
        if (!Block::is_due(current_time_us)) return false;
        
        this->mark_updated(current_time_us);

        const auto& true_state = this->input_.get();

        ReadingT reading = sample(true_state, current_time_us / 1e6);
        reading.set_timestamp(current_time_us);
        reading.timestamp_us = current_time_us;
        
        latency_buffer_.push_back(reading);
        
        while (latency_buffer_.size() > 1) {
            // Use timestamp() (IInterBlockData), NOT timestamp_us (sensor field, never set here)
            uint64_t age = current_time_us - latency_buffer_.front().timestamp();
            if (age > latency_us_ + 2 * this->update_period_us_) {
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

    bool initialize() override { return true; }

    void set_latency(uint32_t latency_us) { latency_us_ = latency_us; }
    uint32_t latency() const { return latency_us_; }

protected:
    virtual ReadingT sample(const TrueStateT& true_state, uint64_t current_time_us) = 0;

    double gaussian_noise(double stddev) {
        return std::normal_distribution<double>(0.0, stddev)(rng_);
    }

    uint32_t latency_us_;
    std::deque<ReadingT> latency_buffer_;
    bool new_reading_available_ = false;
    std::mt19937 rng_;
};

} // namespace sim