#pragma once

#include "../core/block.hpp"
#include "../data/state.hpp"
#include "../data/sensor_reading.hpp"
#include "../../../shared/observer.h"

namespace sim {

/**
 * Base class for state observer/estimator blocks in simulation.
 * 
 * Inherits from:
 *   - Block: for sim scheduling and output callbacks
 *   - shared::IObserver<State>: for unified observer interface
 * 
 * Implementations range from pass-through (for BNO055) to full EKF.
 */
class Observer : public Block, public shared::IObserver<State> {
public:
    Observer(const std::string& name, double update_rate_hz)
        : Block(name, update_rate_hz)
    {}

    // === shared::IObserver interface ===
    
    void feed_imu(const sensors::ImuMeasurement& imu) override {
        // Default: subclasses override as needed
        (void)imu;
    }
    
    void feed_mag(const sensors::MagMeasurement& mag) override { (void)mag; }
    void feed_baro(const sensors::BaroMeasurement& baro) override { (void)baro; }
    void feed_gnss(const sensors::GnssMeasurement& gnss) override { (void)gnss; }
    
    State output() const override { return estimated_state_; }
    
    void reset(const State& initial) override {
        estimated_state_ = initial;
    }
    
    // === Sim-specific sensor feeds (use sim reading wrappers) ===
    
    virtual void feed_imu(const ImuReading& reading) { 
        feed_imu(reading.data);  // Delegate to IObserver interface
    }
    virtual void feed_attitude(const AttitudeReading& reading) { (void)reading; }
    virtual void feed_gps(const GpsReading& reading) { (void)reading; }
    virtual void feed_baro(const BaroReading& reading) { 
        feed_baro(reading.data);  // BaroReading wraps sensors::BaroMeasurement
    }
    virtual void feed_mag(const MagReading& reading) {
        feed_mag(reading.data);  // MagReading wraps sensors::MagMeasurement
    }

    // Legacy accessor (prefer output())
    const State& estimated_state() const { return estimated_state_; }

protected:
    State estimated_state_;
};

/**
 * Pass-through observer for use with AHRS like BNO055.
 * Just forwards the attitude reading as the state estimate.
 * Position/velocity remain at zero (or could be fed from GPS).
 */
class PassthroughObserver : public Observer {
public:
    explicit PassthroughObserver(const std::string& name = "passthrough_observer")
        : Observer(name, 0.0)  // Updates whenever fed data
    {}

    void feed_attitude(const AttitudeReading& reading) override {
        last_attitude_ = reading;
        has_attitude_ = true;
    }

    void feed_gps(const GpsReading& reading) override {
        last_gps_ = reading;
        has_gps_ = true;
    }

    bool update(double current_time_s) override {
        if (!has_attitude_) return false;

        // Copy attitude
        estimated_state_.attitude = last_attitude_.attitude;
        estimated_state_.angular_velocity = last_attitude_.angular_velocity;

        // If we have GPS, use it for velocity
        // (Position would need coordinate transforms)
        if (has_gps_) {
            estimated_state_.velocity = last_gps_.velocity_ned();
        }

        estimated_state_.set_timestamp(current_time_s);
        mark_updated(current_time_s);
        notify_output(estimated_state_);

        return true;
    }

private:
    AttitudeReading last_attitude_;
    GpsReading last_gps_;
    bool has_attitude_ = false;
    bool has_gps_ = false;
};

} // namespace sim
