#ifndef SHARED_OBSERVER_H
#define SHARED_OBSERVER_H

#include "sensors/sensor_readings.h"
#include "types/state.h"

namespace shared {

/**
 * Abstract observer interface templated on output state type.
 *
 * StateT must provide position, velocity, attitude, angular_velocity fields.
 * Implementations include EKF16d (INS+GNSS+baro+mag), future AHRS-only
 * observers, etc.
 *
 * In sim, concrete observers are wrapped by a Block for scheduling.
 * In embedded, observers are called directly from sensor callbacks.
 */
template<typename StateT>
class IObserver {
public:
    virtual ~IObserver() = default;

    // Sensor input methods — implementations consume what they need.
    virtual void feed_imu(const sensors::ImuMeasurement& imu) = 0;
    virtual void feed_mag(const sensors::MagMeasurement& mag) { (void)mag; }
    virtual void feed_baro(const sensors::BaroMeasurement& baro) { (void)baro; }
    virtual void feed_gnss(const sensors::GnssMeasurement& gnss) { (void)gnss; }

    // Build and return the current state estimate. Const — must not mutate
    // filter state; all mutations happen in feed_*().
    virtual StateT output() const = 0;

    // Reset observer to the given initial state.
    virtual void reset(const StateT& initial) = 0;
};

// Common instantiations
using ITrueStateObserver = IObserver<TrueState>;
using INavObserver       = IObserver<NavigationState>;

} // namespace shared

#endif // SHARED_OBSERVER_H
