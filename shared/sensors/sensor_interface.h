#ifndef SHARED_SENSOR_INTERFACE_H
#define SHARED_SENSOR_INTERFACE_H

#include <stdint.h>

namespace sensors {

/**
 * Non-template base interface for all sensors.
 * Enables polymorphic handling of sensors with different reading types.
 */
class ISensor {
public:
    virtual ~ISensor() = default;

    // Initialize the sensor hardware/simulation
    virtual bool initialize() = 0;

    // Check if sensor is due for an update
    virtual bool is_due(uint64_t current_time_us) = 0;

    // Update the sensor reading
    virtual bool update(uint64_t current_time_us) = 0;

    // Get the sensor name
    virtual const char* name() const = 0;
};

}  // namespace sensors

#endif  // SHARED_SENSOR_INTERFACE_H
