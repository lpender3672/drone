
#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include "sensor_base.h"
#include <MPU6050.h>
#include <ekf16d.h>

class ImuSensor : public SensorBase {
private:
    MPU6050 mpu_;
    IEKF* ekf_;
    static constexpr double G_ACCEL = 9.80665;
    // MPU-6050 default sensitivity values
    static constexpr float ACCEL_SCALE = 16384.0f;  // ±2g range: 16384 LSB/g
    static constexpr float GYRO_SCALE = 131.0f;     // ±250°/s range: 131 LSB/(°/s)
    static constexpr float DEG2RAD = 0.017453292519943f;

public:
    ImuSensor(IEKF* ekf, uint32_t interval_ms = 10)
        : SensorBase("IMU", interval_ms), mpu_(), ekf_(ekf) {}

    MPU6050* mpu() { return &mpu_; }

    bool initialize() override {
        Wire.begin();
        mpu_.initialize();
        
        if (!mpu_.testConnection()) {
            Serial.println("MPU6050 init failed");
            return false;
        }
        delay(100);
        
        // Configure MPU-6050
        mpu_.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2g
        mpu_.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250°/s
        mpu_.setDLPFMode(MPU6050_DLPF_BW_42);              // Low-pass filter
        
        Serial.println("MPU6050 initialized");
        return true;
    }

    void update() override {
        startTiming();

        int16_t ax_raw, ay_raw, az_raw;
        int16_t gx_raw, gy_raw, gz_raw;
        mpu_.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

        // Convert to physical units
        // Accelerometer: raw -> g -> m/s²
        float ax = (float)ax_raw / ACCEL_SCALE * G_ACCEL;
        float ay = (float)ay_raw / ACCEL_SCALE * G_ACCEL;
        float az = (float)az_raw / ACCEL_SCALE * G_ACCEL;
        
        // Gyroscope: raw -> °/s -> rad/s
        float gx = (float)gx_raw / GYRO_SCALE * DEG2RAD;
        float gy = (float)gy_raw / GYRO_SCALE * DEG2RAD;
        float gz = (float)gz_raw / GYRO_SCALE * DEG2RAD;

        static uint32_t last_time = 0;
        uint32_t now = millis();
        float dt = (now - last_time) * 0.001f;
        last_time = now;

        const Eigen::Vector3d acc_g(ax, ay, az);
        const Eigen::Vector3d gyro_rads(gx, gy, gz);

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
            msg.acc = Eigen::Vector3d(ax, ay, az) / G_ACCEL;
            msg.gyro = Eigen::Vector3d(gx, gy, gz);
            //ekf_->predict(msg, dt);
        }

        endTiming();

        saveValueIfEnabled(now, sample);
    }
};

#endif
