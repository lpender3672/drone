#ifndef SHARED_SENSOR_BASE_H
#define SHARED_SENSOR_BASE_H

#include "sensor_interface.h"
#include "sensor_readings.h"
#include <stdint.h>
#include <optional>

namespace sensors {

/**
 * Base class for all sensors across all platforms.
 * Provides common interface: initialization, updating, timing, and reading access.
 *
 * Template parameter ReadingType should be a sensor reading type (e.g., ImuReading)
 */
template<typename ReadingType>
class Sensor : public ISensor {
protected:
    const char* name_;
    uint64_t interval_us_;       // Update interval in microseconds
    uint64_t last_update_us_;    // Last update timestamp
    uint64_t next_due_us_;       // Next scheduled update
    bool has_next_due_;
    bool initialized_;
    ReadingType latest_reading_;
    bool new_reading_available_ = false;

public:
    Sensor(const char* name, uint64_t interval_us)
        : name_(name), interval_us_(interval_us), last_update_us_(0),
          next_due_us_(0), has_next_due_(false), initialized_(false) {}

    virtual ~Sensor() {}

    // Initialize the sensor hardware/simulation
    // Returns true on success, false on failure
    bool initialize() override = 0;

    // Check if sensor is due for an update at current_time_us
    // Returns true if enough time has passed since last update
    bool is_due(uint64_t current_time_us) override {
        if (!has_next_due_) {
            next_due_us_ = current_time_us + interval_us_;
            has_next_due_ = true;
            return true;
        }

        if (current_time_us >= next_due_us_) {
            do {
                next_due_us_ += interval_us_;
            } while (next_due_us_ <= current_time_us);
            return true;
        }
        return false;
    }

    // Update the sensor reading (implemented by derived classes)
    bool update(uint64_t current_time_us) override = 0;

    // Get the latest sensor reading
    // Returns nullopt if no reading is available
    virtual std::optional<ReadingType> get_reading() const {
        return latest_reading_;
    }

    // Check if a new reading is available since last call
    virtual bool has_new_reading() const {
        return new_reading_available_;
    }

    // Clear the new reading flag
    virtual void clear_new_reading_flag() {
        new_reading_available_ = false;
    }

    // Returns the latest reading and clears the new-reading flag atomically.
    // Returns nullopt if no new reading is available since the last consume.
    std::optional<ReadingType> consume_reading() {
        if (!new_reading_available_) return std::nullopt;
        new_reading_available_ = false;
        return latest_reading_;
    }

    void log_to(ILogger& logger, uint32_t now_ms) override {
        auto r = consume_reading();
        if (!r) return;
        double arr[ReadingType::DataSize];
        r->to_array(arr);
        logger.write(now_ms, arr, sizeof(arr));
    }

    // Accessors
    const char* name() const override { return name_; }
    uint32_t interval_us() const { return interval_us_; }
    bool is_initialized() const { return initialized_; }
    void set_initialized(bool init) { initialized_ = init; }

    void set_interval_ms(uint32_t interval_ms) { interval_us_ = (uint64_t)interval_ms * 1000; }
    void set_interval_us(uint64_t interval_us) { interval_us_ = interval_us; }
};

}  // namespace sensors

#endif  // SHARED_SENSOR_BASE_H
