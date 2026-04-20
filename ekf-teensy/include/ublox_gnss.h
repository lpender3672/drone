#ifndef EKF_TEENSY_UBLOX_GNSS_H
#define EKF_TEENSY_UBLOX_GNSS_H

#include <sensor_base.h>
#include <sensor_readings.h>
#include <sensor_constants.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <observer.h>

#include "teensy_sensor_logger.h"

/**
 * u-blox GNSS sensor implementation for Teensy.
 * Provides position and velocity measurements from u-blox GPS modules.
 */
class UbloxGnss : public sensors::Sensor<sensors::GnssReading>, public sensors::TeensySensorLogger {
private:
    SFE_UBLOX_GNSS gnss_;
    shared::IObserverWithBiases* observer_;

    // Noise parameters
    double pos_std_ = 2.0;  // meters
    double vel_std_ = 0.1;  // m/s

public:
    UbloxGnss(shared::IObserverWithBiases* observer, uint32_t interval_ms = 100)
        : Sensor<sensors::GnssReading>("GNSS", (uint64_t)interval_ms * 1000),
          TeensySensorLogger("GNSS"),
          observer_(observer) {}

    bool initialize() override {
        if (!gnss_.begin()) {
            Serial.println("GNSS init failed");
            initialized_ = false;
            return false;
        }
        gnss_.setI2COutput(COM_TYPE_UBX);
        gnss_.setNavigationFrequency(10);
        gnss_.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
        Serial.println("GNSS initialized");
        initialized_ = true;
        return true;
    }

    bool update(uint64_t current_time_us_64) override {
        uint32_t current_time_us = static_cast<uint32_t>(current_time_us_64);
        startTiming();

        if (!gnss_.getPVT()) {
            endTiming();
            return false;
        }

        uint8_t fix = gnss_.getFixType();
        if (fix < 3) {
            endTiming();
            return false;
        }

        const double lat = gnss_.getLatitude() * 1e-7;
        const double lon = gnss_.getLongitude() * 1e-7;
        const double alt = gnss_.getAltitude() * 1e-3;

        // Update the standardized reading structure
        latest_reading_.timestamp_us = current_time_us;
        latest_reading_.latitude_deg = lat;
        latest_reading_.longitude_deg = lon;
        latest_reading_.altitude_m = alt;
        latest_reading_.velocity_ned = Eigen::Vector3d(
            gnss_.getNedNorthVel() * 1e-3,
            gnss_.getNedEastVel() * 1e-3,
            gnss_.getNedDownVel() * 1e-3);
        latest_reading_.fix_type = fix;
        latest_reading_.num_satellites = gnss_.getSIV();
        latest_reading_.hdop = gnss_.getHorizontalDOP() * 0.01f;
        latest_reading_.vdop = gnss_.getVerticalDOP() * 0.01f;
        latest_reading_.h_accuracy_m = gnss_.getHorizontalAccuracy() * 1e-4f;  // 0.1mm units → m
        latest_reading_.v_accuracy_m = gnss_.getVerticalAccuracy()   * 1e-4f;
        latest_reading_.valid = true;

        new_reading_available_ = true;

        observer_->feed_gnss(latest_reading_);

        // Log data to SD card if enabled
        struct GnssLogSample {
            float pos[3];
            float vel[3];
            uint8_t fix;
        } sample;

        sample.pos[0] = static_cast<float>(lat);
        sample.pos[1] = static_cast<float>(lon);
        sample.pos[2] = static_cast<float>(alt);
        sample.vel[0] = static_cast<float>(latest_reading_.velocity_ned.x());
        sample.vel[1] = static_cast<float>(latest_reading_.velocity_ned.y());
        sample.vel[2] = static_cast<float>(latest_reading_.velocity_ned.z());
        sample.fix = fix;

        endTiming();

        uint32_t now_ms = current_time_us / 1000;
        saveValueIfEnabled(now_ms, sample);
        markUpdated(current_time_us);
        return true;
    }

    void setPosStd(double std) { pos_std_ = std; }
    void setVelStd(double std) { vel_std_ = std; }
};

#endif  // EKF_TEENSY_UBLOX_GNSS_H
