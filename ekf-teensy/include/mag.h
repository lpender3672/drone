#ifndef SENSOR_MAG_H
#define SENSOR_MAG_H

#include "sensor_base.h"
#include <Adafruit_BNO055.h>
#include <ekf.h>

class MagSensor : public SensorBase {
private:
    Adafruit_BNO055* bno_;  // Shared with IMU
    IEKF* ekf_;
    double mag_std_ = 0.5;  // uT

public:
    MagSensor(Adafruit_BNO055* bno, IEKF* ekf, uint32_t interval_ms = 20)
        : SensorBase("Mag", interval_ms), bno_(bno), ekf_(ekf) {}

    bool initialize() override {
        // BNO055 initialized by IMU sensor
        Serial.println("Mag sensor ready (shares BNO055)");
        return true;
    }

    void update() override {
        startTiming();

        sensors_event_t mag;
        bno_->getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);

        Eigen::Vector3d mag_body(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * (mag_std_ * mag_std_);

        ekf_->update_magnetometer(mag_body, R);

        endTiming();
    }

    void setMagStd(double std) { mag_std_ = std; }
};

#endif