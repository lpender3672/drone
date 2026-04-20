#ifndef EKF_TEENSY_BNO055_MAG_H
#define EKF_TEENSY_BNO055_MAG_H

#include <sensor_base.h>
#include <sensor_readings.h>
#include <Adafruit_BNO055.h>
#include <observer.h>

#include "teensy_sensor_logger.h"

/**
 * BNO055 magnetometer sensor implementation for Teensy.
 * Shares the BNO055 chip with the IMU sensor.
 */
class BNO055Mag : public sensors::Sensor<sensors::MagReading>, public sensors::TeensySensorLogger {
private:
    Adafruit_BNO055* bno_;  // Shared with IMU
    shared::IObserverWithBiases* observer_;

public:
    BNO055Mag(Adafruit_BNO055* bno, shared::IObserverWithBiases* observer, uint32_t interval_ms = 20)
        : Sensor<sensors::MagReading>("Mag", (uint64_t)interval_ms * 1000),
          TeensySensorLogger("Mag"),
          bno_(bno), observer_(observer) {}

    bool initialize() override {
        // BNO055 initialized by IMU sensor
        Serial.println("Mag sensor ready (shares BNO055)");
        initialized_ = true;
        return true;
    }

    bool update(uint64_t current_time_us_64) override {
        uint32_t current_time_us = static_cast<uint32_t>(current_time_us_64);
        startTiming();

        sensors_event_t mag;
        bno_->getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);

        // Update the standardized reading structure
        latest_reading_.timestamp_us = current_time_us;
        latest_reading_.field = Eigen::Vector3d(
            mag.magnetic.x,
            mag.magnetic.y,
            mag.magnetic.z);
        latest_reading_.valid = true;

        new_reading_available_ = true;

        observer_->feed_mag(latest_reading_);

        // Log data to SD card if enabled
        struct MagLogSample {
            float mag_uT[3];
        } sample;

        sample.mag_uT[0] = mag.magnetic.x;
        sample.mag_uT[1] = mag.magnetic.y;
        sample.mag_uT[2] = mag.magnetic.z;

        endTiming();

        uint32_t now_ms = current_time_us / 1000;
        saveValueIfEnabled(now_ms, sample);
        markUpdated(current_time_us);
        return true;
    }

};

#endif  // EKF_TEENSY_BNO055_MAG_H
