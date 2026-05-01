#pragma once

#include "block.hpp"
#include "../data/state.hpp"
#include <vector>
#include <memory>

namespace sim {

/**
 * Simulation manager.
 * Coordinates block execution and timing.
 * All times are in microseconds (uint64_t) throughout.
 */
class Simulation {
public:
    explicit Simulation(uint32_t dt_us = 1000)
        : dt_us_(dt_us)
        , current_time_us_(0)
    {}

    // Add block (simulation takes ownership). Returns raw pointer for wiring.
    template<typename T>
    T* add_block(std::unique_ptr<T> block) {
        T* ptr = block.get();
        blocks_.push_back(std::move(block));
        return ptr;
    }

    // Run for specified duration
    void run(double duration_s) {
        uint64_t end_us = current_time_us_ + static_cast<uint64_t>(duration_s * 1e6);
        while (current_time_us_ < end_us) {
            step();
        }
    }

    // Single step: advance each due block then increment time
    void step() {
        for (auto& block : blocks_) {
            if (block->is_due(current_time_us_))
                block->update(current_time_us_);
        }
        current_time_us_ += dt_us_;
    }

    void reset() { current_time_us_ = 0; }

    uint64_t current_time_us() const { return current_time_us_; }
    double   current_time_s()  const { return static_cast<double>(current_time_us_) * 1e-6; }

private:
    std::vector<std::unique_ptr<Block>> blocks_;

    uint32_t dt_us_;
    uint64_t current_time_us_;
};

} // namespace sim
