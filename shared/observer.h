#ifndef SHARED_OBSERVER_H
#define SHARED_OBSERVER_H

#include "sensors/sensor_readings.h"
#include "types/state.h"

namespace shared {

/**
 * Abstract observer interface templated on output state type.
 * 
 * StateT must provide position, velocity, attitude, angular_velocity fields.
 * Implementations include:
 *   - EKF16d (embedded/sim) -> outputs StateWithBiases
 *   - PassthroughObserver (sim with AHRS) -> outputs StateBase
 * 
 * In sim, concrete observers also inherit from Block to get scheduling.
 * In embedded, observers are called directly from sensor callbacks.
 */
template<typename StateT>
class IObserver {
public:
    virtual ~IObserver() = default;
    
    // Sensor input methods - implementations consume what they need
    virtual void feed_imu(const sensors::ImuMeasurement& imu) = 0;
    virtual void feed_mag(const sensors::MagMeasurement& mag) { (void)mag; }
    virtual void feed_baro(const sensors::BaroMeasurement& baro) { (void)baro; }
    virtual void feed_gnss(const sensors::GnssMeasurement& gnss) { (void)gnss; }
    
    /**
     * Build and return the current state estimate.
     * 
     * This combines internal filter state (position, velocity, attitude, biases)
     * with cached sensor data (angular velocity from last IMU).
     * 
     * The output() method is const - it should not modify filter state.
     * State mutations happen only in feed_*() and predict/update steps.
     */
    virtual StateT output() const = 0;
    
    /**
     * Reset observer to initial state.
     */
    virtual void reset(const StateT& initial) = 0;
};

// Common instantiations
using IObserverBase = IObserver<StateBase>;
using IObserverWithBiases = IObserver<StateWithBiases>;

} // namespace shared

#endif // SHARED_OBSERVER_H
