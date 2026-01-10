
#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include "sensor_base.h"
#include <Adafruit_BNO055.h>
#include <ekf16d.h>

class ImuSensor : public SensorBase {
private:
    Adafruit_BNO055 bno_;
    IEKF* ekf_;
    static constexpr double G_ACCEL = 9.80665;

public:
    ImuSensor(IEKF* ekf, uint32_t interval_ms = 10)
        : SensorBase("IMU", interval_ms), bno_(55, 0x28), ekf_(ekf) {}

    Adafruit_BNO055* bno() { return &bno_; }

    bool initialize() override {
        if (!bno_.begin()) {
            Serial.println("BNO055 init failed");
            return false;
        }
        delay(100);
        bno_.setExtCrystalUse(true);
        bno_.setMode(OPERATION_MODE_AMG);
        Serial.println("BNO055 initialized");
        return true;
    }

    void update() override {
        startTiming();

        sensors_event_t accel, gyro;
        bno_.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        bno_.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);

        static uint32_t last_time = 0;
        uint32_t now = millis();
        float dt = (now - last_time) * 0.001f;
        last_time = now;

        const Eigen::Vector3d acc_g(accel.acceleration.x,
                                   accel.acceleration.y,
                                   accel.acceleration.z);
        const Eigen::Vector3d gyro_rads(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);

        struct ImuLogSample {
            float acc_g[3];
            float gyro_rads[3];
            float dt;
        } sample;

        sample.acc_g[0] = static_cast<float>(acc_g.x() / G_ACCEL);
        sample.acc_g[1] = static_cast<float>(acc_g.y() / G_ACCEL);
        sample.acc_g[2] = static_cast<float>(acc_g.z() / G_ACCEL);
        sample.gyro_rads[0] = static_cast<float>(gyro_rads.x());
        sample.gyro_rads[1] = static_cast<float>(gyro_rads.y());
        sample.gyro_rads[2] = static_cast<float>(gyro_rads.z());
        sample.dt = dt;

        if (dt > 0.0f && dt < 0.1f) {
            ImuMeasurement msg;
            msg.acc = Eigen::Vector3d(accel.acceleration.x, 
                                       accel.acceleration.y, 
                                       accel.acceleration.z) / G_ACCEL;
            msg.gyro = Eigen::Vector3d(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
            //ekf_->predict(msg, dt);
        }

        endTiming();

        saveValueIfEnabled(now, sample);
    }
};

#endif
