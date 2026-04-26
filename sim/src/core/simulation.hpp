#pragma once

#include "block.hpp"
#include "../data/state.hpp"
#include <vector>
#include <memory>
#include <map>
#include <functional>
#include <fstream>
#include <iomanip>

namespace sim {

/**
 * Data logger for recording simulation outputs.
 * Attaches as callbacks to blocks.
 */
class DataLogger {
public:
    struct LogEntry {
        double time;
        std::string block_name;
        std::string data_type;
        std::vector<std::pair<std::string, double>> values;
    };

    void log(const std::string& block_name, const IInterBlockData& data) {
        LogEntry entry;
        entry.time = data.timestamp();
        entry.block_name = block_name;
        entry.data_type = data.type_name();

        // TOOD

        entries_.push_back(entry);
    }

    // Write to CSV file (one file per block/type combination)
    void write_csv(const std::string& prefix) const {
        std::map<std::string, std::ofstream> files;
        std::map<std::string, bool> headers_written;

        for (const auto& entry : entries_) {
            std::string key = entry.block_name + "_" + entry.data_type;

            if (files.find(key) == files.end()) {
                std::string filename = prefix + key + ".csv";
                files[key].open(filename);
                headers_written[key] = false;
            }

            auto& file = files[key];

            // Write header on first entry
            if (!headers_written[key] && !entry.values.empty()) {
                file << "time";
                for (const auto& [name, _] : entry.values) {
                    file << "," << name;
                }
                file << "\n";
                headers_written[key] = true;
            }

            // Write data
            file << std::fixed << std::setprecision(6) << entry.time;
            for (const auto& [_, value] : entry.values) {
                file << "," << value;
            }
            file << "\n";
        }
    }

    void clear() { entries_.clear(); }
    size_t size() const { return entries_.size(); }

private:
    std::vector<LogEntry> entries_;
};

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

    // Attach logger to all blocks
    void attach_logger(DataLogger* logger) {
        logger_ = logger;
        for (auto& block : blocks_) {
            block->add_output_callback([logger](const std::string& name, const IInterBlockData& data) {
                logger->log(name, data);
            });
        }
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
    DataLogger* logger_ = nullptr;

    uint32_t dt_us_;
    uint64_t current_time_us_;
};

} // namespace sim
