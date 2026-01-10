#ifndef SHARED_MAG_SENSOR_BASE_H
#define SHARED_MAG_SENSOR_BASE_H

#include "sensor_base.h"
#include "sensor_readings.h"

namespace sensors {

/**
 * Base class for magnetometer sensors across all platforms.
 * Provides standard interface for 3-axis magnetic field measurement.
 */
class MagSensorBase : public SensorBase {
protected:
    MagReading latest_reading_;
    
public:
    MagSensorBase(const char* name, uint32_t interval_us)
        : SensorBase(name, interval_us) {}
    
    virtual ~MagSensorBase() {}
    
    // Get the latest magnetometer reading
    const MagReading& get_reading() const { return latest_reading_; }
    
    // Check if a new reading is available since last call
    virtual bool has_new_reading() const = 0;
    
    // Clear the new reading flag
    virtual void clear_new_reading_flag() = 0;
};

}  // namespace sensors

#endif  // SHARED_MAG_SENSOR_BASE_H
