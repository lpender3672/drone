#pragma once

#include "block.hpp"
#include "../data/state.hpp"
#include "../data/control_efforts.hpp"
#include "../data/reference.hpp"
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

    void log(const std::string& block_name, const InterBlockData& data) {
        LogEntry entry;
        entry.time = data.timestamp();
        entry.block_name = block_name;
        entry.data_type = data.type_name();
        
        // Extract values based on type
        if (data.type_name() == "State") {
            const auto& s = static_cast<const State&>(data);
            Vec3 euler = s.euler_angles();
            entry.values = {
                {"x", s.position.x()}, {"y", s.position.y()}, {"z", s.position.z()},
                {"vx", s.velocity.x()}, {"vy", s.velocity.y()}, {"vz", s.velocity.z()},
                {"roll", euler.x()}, {"pitch", euler.y()}, {"yaw", euler.z()},
                {"p", s.angular_velocity.x()}, {"q", s.angular_velocity.y()}, {"r", s.angular_velocity.z()},
                {"qw", s.attitude.w()}, {"qx", s.attitude.x()}, {"qy", s.attitude.y()}, {"qz", s.attitude.z()}
            };
        } else if (auto* efforts = dynamic_cast<const ControlEffortsBase*>(&data)) {
            for (int i = 0; i < efforts->num_channels(); ++i) {
                entry.values.push_back({"e" + std::to_string(i), efforts->effort(i)});
            }
        } else if (data.type_name() == "Reference") {
            const auto& r = static_cast<const Reference&>(data);
            entry.values = {
                {"roll", r.attitude.x()}, {"pitch", r.attitude.y()}, {"yaw", r.attitude.z()},
                {"thrust", r.thrust}
            };
        }
        // Add more types as needed

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
 * Coordinates block execution, timing, and data flow.
 */
class Simulation {
public:
    Simulation(double dt = 0.001)  // Default 1ms timestep
        : dt_(dt)
        , current_time_(0.0)
    {}

    // Add blocks (simulation takes ownership)
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
            block->add_output_callback([logger](const std::string& name, const InterBlockData& data) {
                logger->log(name, data);
            });
        }
    }

    // Run for specified duration
    void run(double duration_s) {
        double end_time = current_time_ + duration_s;
        
        while (current_time_ < end_time) {
            step();
        }
    }

    // Single step
    void step() {
        for (auto& block : blocks_) {
            block->update(current_time_);
        }
        current_time_ += dt_;
    }

    // Reset simulation
    void reset() {
        current_time_ = 0.0;
    }

    double current_time() const { return current_time_; }
    double dt() const { return dt_; }

private:
    std::vector<std::unique_ptr<Block>> blocks_;
    DataLogger* logger_ = nullptr;
    
    double dt_;
    double current_time_;
};

} // namespace sim
