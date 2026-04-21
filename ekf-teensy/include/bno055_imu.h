#ifndef EKF_TEENSY_BNO055_IMU_H
#define EKF_TEENSY_BNO055_IMU_H

#include <sensor_base.h>
#include <sensor_readings.h>
#include <sensor_constants.h>
#include "teensy_sensor_logger.h"
#include <Adafruit_BNO055.h>
#include <observer.h>

/**
 * BNO055 IMU sensor implementation for Teensy.
 * Provides accelerometer and gyroscope readings from the BNO055 chip.
 */
class BNO055Imu : public sensors::Sensor<sensors::ImuReading>, public sensors::SensorTiming {
private:
    Adafruit_BNO055 bno_;
    shared::IObserverWithBiases* observer_;

public:
    BNO055Imu(shared::IObserverWithBiases* observer, uint32_t interval_ms = 10)
        : Sensor<sensors::ImuReading>("IMU", (uint64_t)interval_ms * 1000),
          bno_(55, 0x28), observer_(observer) {}

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

    bool update(uint64_t current_time_us_64) override {
        uint32_t current_time_us = static_cast<uint32_t>(current_time_us_64);
        startTiming();

        sensors_event_t accel, gyro;
        bno_.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        bno_.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);

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

        observer_->feed_imu(latest_reading_);

        endTiming();
        markUpdated(current_time_us);
        return true;
    }
};

#endif  // EKF_TEENSY_BNO055_IMU_H
