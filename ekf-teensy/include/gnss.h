#ifndef SENSOR_GNSS_H
#define SENSOR_GNSS_H

#include "sensor_base.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <ekf.h>

#ifndef DEG_TO_RAD
constexpr double DEG_TO_RAD = M_PI / 180.0;
#endif

class GnssSensor : public SensorBase {
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
    GnssSensor(IEKF* ekf, uint32_t interval_ms = 100)
        : SensorBase("GNSS", interval_ms), ekf_(ekf) {}

    bool initialize() override {
        if (!gnss_.begin()) {
            Serial.println("GNSS init failed");
            return false;
        }
        gnss_.setI2COutput(COM_TYPE_UBX);
        gnss_.setNavigationFrequency(10);
        gnss_.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
        Serial.println("GNSS initialized");
        return true;
    }

    void update() override {
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

        double lat = gnss_.getLatitude() * 1e-7;
        double lon = gnss_.getLongitude() * 1e-7;
        double alt = gnss_.getAltitude() * 1e-3;

        // Set reference on first fix
        if (!ref_set_) {
            lat_ref_ = lat;
            lon_ref_ = lon;
            alt_ref_ = alt;
            ref_set_ = true;
            Serial.printf("GNSS ref set: %.6f, %.6f, %.1f\n", lat, lon, alt);
        }

        Eigen::Vector3d pos_ned(lat, lon, alt);
        Eigen::Vector3d vel_ned(
            gnss_.getNedNorthVel() * 1e-3,
            gnss_.getNedEastVel() * 1e-3,
            gnss_.getNedDownVel() * 1e-3
        );

        // Position covariance (diagonal)
        Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * (pos_std_ * pos_std_);
        Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * (vel_std_ * vel_std_);

        ekf_->update_gnss_position(pos_ned, R_pos);
        ekf_->update_gnss_velocity(vel_ned, R_vel);

        endTiming();
    }

    bool hasReference() const { return ref_set_; }
    void setPosStd(double std) { pos_std_ = std; }
    void setVelStd(double std) { vel_std_ = std; }
};

#endif