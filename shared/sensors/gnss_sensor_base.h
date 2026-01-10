#ifndef SHARED_GNSS_SENSOR_BASE_H
#define SHARED_GNSS_SENSOR_BASE_H

#include "sensor_base.h"
#include "sensor_readings.h"

namespace sensors {

/**
 * Base class for GNSS sensors across all platforms.
 * Provides standard interface for position and velocity measurements.
 */
class GnssSensorBase : public SensorBase {
protected:
    GnssReading latest_reading_;
    
public:
    GnssSensorBase(const char* name, uint32_t interval_us)
        : SensorBase(name, interval_us) {}
    
    virtual ~GnssSensorBase() {}
    
    // Get the latest GNSS reading
    const GnssReading& get_reading() const { return latest_reading_; }
    
    // Check if a new reading is available since last call
    virtual bool has_new_reading() const = 0;
    
    // Clear the new reading flag
    virtual void clear_new_reading_flag() = 0;
};

}  // namespace sensors

#endif  // SHARED_GNSS_SENSOR_BASE_H
