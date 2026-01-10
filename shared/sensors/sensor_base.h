#ifndef SHARED_SENSOR_BASE_H
#define SHARED_SENSOR_BASE_H

#include "sensor_readings.h"
#include <stdint.h>

namespace sensors {

/**
 * Base class for all sensors across all platforms.
 * Provides common interface: initialization, updating, timing.
 */
class SensorBase {
protected:
    const char* name_;
    uint32_t interval_us_;       // Update interval in microseconds
    uint32_t last_update_us_;    // Last update timestamp
    uint32_t next_due_us_;       // Next scheduled update
    bool has_next_due_;
    bool initialized_;
    
public:
    SensorBase(const char* name, uint32_t interval_us)
        : name_(name), interval_us_(interval_us), last_update_us_(0),
          next_due_us_(0), has_next_due_(false), initialized_(false) {}
    
    virtual ~SensorBase() {}
    
    // Initialize the sensor hardware/simulation
    // Returns true on success, false on failure
    virtual bool initialize() = 0;
    
    // Check if sensor is due for an update at current_time_us
    // Returns true if enough time has passed since last update
    virtual bool is_due(uint32_t current_time_us) {
        if (!has_next_due_) {
            next_due_us_ = current_time_us + interval_us_;
            has_next_due_ = true;
            return true;
        }
        
        if ((int32_t)(current_time_us - next_due_us_) >= 0) {
            next_due_us_ += interval_us_;
            return true;
        }
        return false;
    }
    
    // Update the sensor reading (implemented by derived classes)
    virtual void update(uint32_t current_time_us) = 0;
    
    // Accessors
    const char* name() const { return name_; }
    uint32_t interval_us() const { return interval_us_; }
    bool is_initialized() const { return initialized_; }
    void set_initialized(bool init) { initialized_ = init; }
    
    // Set interval in milliseconds (convenience)
    void set_interval_ms(uint32_t interval_ms) {
        interval_us_ = interval_ms * 1000;
    }
    
    // Set interval in microseconds
    void set_interval_us(uint32_t interval_us) {
        interval_us_ = interval_us;
    }
};

}  // namespace sensors

#endif  // SHARED_SENSOR_BASE_H
