#ifndef EKF_TEENSY_BNO055_IMU_H
#define EKF_TEENSY_BNO055_IMU_H

#include <sensor_base.h>
#include <sensor_readings.h>
#include "teensy_sensor_logger.h"
#include <Adafruit_BNO055.h>
#include <ekf.h>

/**
 * BNO055 IMU sensor implementation for Teensy.
 * Provides accelerometer and gyroscope readings from the BNO055 chip.
 */
class BNO055Imu : public sensors::Sensor<sensors::ImuReading>, public sensors::TeensySensorLogger {
private:
    Adafruit_BNO055 bno_;
    IEKF* ekf_;
    static constexpr double G_ACCEL = 9.80665;

public:
    BNO055Imu(IEKF* ekf, uint32_t interval_ms = 10)
        : Sensor<sensors::ImuReading>("IMU", interval_ms * 1000), 
          TeensySensorLogger("IMU", interval_ms * 1000),
          bno_(55, 0x28), ekf_(ekf) {}

    // Get access to the underlying BNO055 object (for sharing with magnetometer)

    bool is_due(uint32_t current_time_us) override {
        return TeensySensorLogger::is_due(current_time_us);
    }

    Adafruit_BNO055* bno() { return &bno_; }

    bool initialize() override {
        if (!bno_.begin()) {
            Serial.println("BNO055 init failed");
            initialized_ = false;
            return false;
        }
        delay(100);
        bno_.setExtCrystalUse(true);
        bno_.setMode(OPERATION_MODE_AMG);
        Serial.println("BNO055 initialized");
        initialized_ = true;
        return true;
    }

    void update(uint32_t current_time_us) override {
        startTiming();

        sensors_event_t accel, gyro;
        bno_.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        bno_.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);

        static uint32_t last_time_us = 0;
        if (last_time_us == 0) last_time_us = current_time_us;
        
        float dt = (current_time_us - last_time_us) * 1e-6f;
        last_time_us = current_time_us;

        // Update the standardized reading structure
        latest_reading_.timestamp_us = current_time_us;
        latest_reading_.acc = Eigen::Vector3d(
            accel.acceleration.x,
            accel.acceleration.y,
            accel.acceleration.z);
        latest_reading_.gyro = Eigen::Vector3d(
            gyro.gyro.x,
            gyro.gyro.y,
            gyro.gyro.z);
        latest_reading_.valid = true;

        new_reading_available_ = true;

        // Update EKF if valid time step
        if (dt > 0.0f && dt < 0.1f) {
            ImuMeasurement msg;
            msg.acc = latest_reading_.acc / G_ACCEL;  // Convert to g's for EKF
            msg.gyro = latest_reading_.gyro;
            // ekf_->predict(msg, dt);  // Uncomment when needed
        }

        // Log data to SD card if enabled
        struct ImuLogSample {
            float acc_g[3];
            float gyro_rads[3];
            float dt;
        } sample;

        sample.acc_g[0] = accel.acceleration.x / G_ACCEL;
        sample.acc_g[1] = accel.acceleration.y / G_ACCEL;
        sample.acc_g[2] = accel.acceleration.z / G_ACCEL;
        sample.gyro_rads[0] = gyro.gyro.x;
        sample.gyro_rads[1] = gyro.gyro.y;
        sample.gyro_rads[2] = gyro.gyro.z;
        sample.dt = dt;

        endTiming();

        uint32_t now_ms = current_time_us / 1000;
        saveValueIfEnabled(now_ms, sample);
        markUpdated(current_time_us);
    }
};

#endif  // EKF_TEENSY_BNO055_IMU_H
