#ifndef EKF_TEENSY_UBLOX_GNSS_H
#define EKF_TEENSY_UBLOX_GNSS_H

#include <sensor_base.h>
#include <sensor_readings.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <ekf.h>

#include "teensy_sensor_logger.h"

#ifndef DEG_TO_RAD
constexpr double DEG_TO_RAD = M_PI / 180.0;
#endif

/**
 * u-blox GNSS sensor implementation for Teensy.
 * Provides position and velocity measurements from u-blox GPS modules.
 */
class UbloxGnss : public sensors::Sensor<sensors::GnssReading>, public sensors::TeensySensorLogger {
private:
    SFE_UBLOX_GNSS gnss_;
    IEKF* ekf_;
    
    // Reference position for local frame
    double lat_ref_ = 0.0, lon_ref_ = 0.0, alt_ref_ = 0.0;
    bool ref_set_ = false;
    
    // Noise parameters
    double pos_std_ = 2.0;  // meters
    double vel_std_ = 0.1;  // m/s

public:
    UbloxGnss(IEKF* ekf, uint32_t interval_ms = 100)
        : Sensor<sensors::GnssReading>("GNSS", interval_ms * 1000),
          TeensySensorLogger("GNSS", interval_ms * 1000),
          ekf_(ekf) {}

    bool is_due(uint32_t current_time_us) override {
        return TeensySensorLogger::is_due(current_time_us);
    }

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

    void update(uint32_t current_time_us) override {
        startTiming();

        if (!gnss_.getPVT()) {
            endTiming();
            return;
        }

        uint8_t fix = gnss_.getFixType();
        if (fix < 3) {
            endTiming();
            return;
        }

        const double lat = gnss_.getLatitude() * 1e-7;
        const double lon = gnss_.getLongitude() * 1e-7;
        const double alt = gnss_.getAltitude() * 1e-3;

        // Set reference on first fix
        if (!ref_set_) {
            lat_ref_ = lat;
            lon_ref_ = lon;
            alt_ref_ = alt;
            ref_set_ = true;
            Serial.printf("GNSS ref set: %.6f, %.6f, %.1f\n", lat, lon, alt);
        }

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
        latest_reading_.valid = true;

        new_reading_available_ = true;

        // Update EKF
        Eigen::Vector3d pos_ned(lat, lon, alt);
        const Eigen::Vector3d& vel_ned = latest_reading_.velocity_ned;

        // Position covariance (diagonal)
        Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * (pos_std_ * pos_std_);
        Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * (vel_std_ * vel_std_);

        ekf_->update_gnss_position(pos_ned, R_pos);
        ekf_->update_gnss_velocity(vel_ned, R_vel);

        // Log data to SD card if enabled
        struct GnssLogSample {
            float pos[3];
            float vel[3];
            uint8_t fix;
        } sample;

        sample.pos[0] = static_cast<float>(pos_ned.x());
        sample.pos[1] = static_cast<float>(pos_ned.y());
        sample.pos[2] = static_cast<float>(pos_ned.z());
        sample.vel[0] = static_cast<float>(vel_ned.x());
        sample.vel[1] = static_cast<float>(vel_ned.y());
        sample.vel[2] = static_cast<float>(vel_ned.z());
        sample.fix = fix;

        endTiming();

        uint32_t now_ms = current_time_us / 1000;
        saveValueIfEnabled(now_ms, sample);
        markUpdated(current_time_us);
    }

    bool hasReference() const { return ref_set_; }
    void setPosStd(double std) { pos_std_ = std; }
    void setVelStd(double std) { vel_std_ = std; }
};

#endif  // EKF_TEENSY_UBLOX_GNSS_H
