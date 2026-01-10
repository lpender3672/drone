#ifndef SHARED_BARO_SENSOR_BASE_H
#define SHARED_BARO_SENSOR_BASE_H

#include "sensor_base.h"
#include "sensor_readings.h"

namespace sensors {

/**
 * Base class for barometer sensors across all platforms.
 * Provides standard interface for pressure and altitude measurements.
 */
class BaroSensorBase : public SensorBase {
protected:
    BaroReading latest_reading_;
    
public:
    BaroSensorBase(const char* name, uint32_t interval_us)
        : SensorBase(name, interval_us) {}
    
    virtual ~BaroSensorBase() {}
    
    // Get the latest barometer reading
    const BaroReading& get_reading() const { return latest_reading_; }
    
    // Check if a new reading is available since last call
    virtual bool has_new_reading() const = 0;
    
    // Clear the new reading flag
    virtual void clear_new_reading_flag() = 0;
    
    // Set reference pressure for altitude calculation
    virtual void set_reference_pressure(float pressure_pa) = 0;
};

}  // namespace sensors

#endif  // SHARED_BARO_SENSOR_BASE_H
