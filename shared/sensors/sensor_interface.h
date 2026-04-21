#ifndef SHARED_SENSOR_INTERFACE_H
#define SHARED_SENSOR_INTERFACE_H

#include <stdint.h>

namespace sensors {

class ILogger {
public:
    virtual ~ILogger() = default;
    virtual void write(uint32_t now_ms, const void* data, size_t len) = 0;
};

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

    // Log latest reading to the provided logger; no-op if no new reading.
    // Default does nothing — sensors that don't log (e.g. sim) can ignore it.
    virtual void log_to(ILogger& logger, uint32_t now_ms) {}
};

}  // namespace sensors

#endif  // SHARED_SENSOR_INTERFACE_H
