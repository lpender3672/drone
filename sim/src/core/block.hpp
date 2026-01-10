#pragma once

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include "inter_block_data.hpp"

namespace sim {

/**
 * Base class for all simulation blocks.
 * Provides common interface for update timing, callbacks, and identification.
 */
class Block {
public:
    // Callback signature: (block_name, data)
    using OutputCallback = std::function<void(const std::string&, const InterBlockData&)>;

    explicit Block(const std::string& name, double update_rate_hz = 0.0)
        : name_(name)
        , update_rate_hz_(update_rate_hz)
        , update_period_s_(update_rate_hz > 0.0 ? 1.0 / update_rate_hz : 0.0)
        , last_update_time_s_(-1e9)  // Ensure first update runs
    {}

    virtual ~Block() = default;

    // Core interface - override in derived classes
    // Returns true if the block actually ran (was due for update)
    virtual bool update(double current_time_s) = 0;

    // Block identification
    const std::string& name() const { return name_; }
    
    // Timing
    double update_rate_hz() const { return update_rate_hz_; }
    double update_period_s() const { return update_period_s_; }
    
    // Check if block is due for update (0 rate = every call)
    bool is_due(double current_time_s) const {
        if (update_rate_hz_ <= 0.0) return true;
        return (current_time_s - last_update_time_s_) >= update_period_s_ - 1e-9;
    }

    // Callback registration for output instrumentation
    void add_output_callback(OutputCallback cb) {
        output_callbacks_.push_back(std::move(cb));
    }

    void clear_callbacks() {
        output_callbacks_.clear();
    }

protected:
    // Call this from derived classes when producing output
    void notify_output(const InterBlockData& data) {
        for (auto& cb : output_callbacks_) {
            cb(name_, data);
        }
    }

    // Update timing bookkeeping - call at start of update()
    void mark_updated(double current_time_s) {
        last_update_time_s_ = current_time_s;
    }

    std::string name_;
    double update_rate_hz_;
    double update_period_s_;
    double last_update_time_s_;
    
    std::vector<OutputCallback> output_callbacks_;
};

} // namespace sim
