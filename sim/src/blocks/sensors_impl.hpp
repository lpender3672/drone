#pragma once

#include "sim_sensor.hpp"

namespace sim {

/**
 * IMU sensor model.
 * Outputs accelerometer and gyroscope readings with noise and bias.
 */
class ImuSensor : public SimSensor<ImuReading> {
public:
    struct NoiseParams {
        // Gyroscope
        double gyro_noise_stddev = 0.001;    // [rad/s] white noise
        double gyro_bias_stddev = 0.0001;    // [rad/s] bias random walk
        
        // Accelerometer  
        double accel_noise_stddev = 0.05;    // [m/s²] white noise
        double accel_bias_stddev = 0.001;    // [m/s²] bias random walk
    };

    ImuSensor(const std::string& name = "imu", 
              double update_rate_hz = 1000.0, 
              double latency_s = 0.001)
        : SimSensor<ImuReading>(name, update_rate_hz, latency_s)
    {}

    void set_noise_params(const NoiseParams& params) { noise_params_ = params; }

    void reset() {
        gyro_bias_ = Vec3::Zero();
        accel_bias_ = Vec3::Zero();
    }

protected:
    ImuReading sample(const State& true_state, double /*current_time_s*/) override {
        ImuReading reading;

        // Random walk on biases
        gyro_bias_ += Vec3(
            gaussian_noise(noise_params_.gyro_bias_stddev),
            gaussian_noise(noise_params_.gyro_bias_stddev),
            gaussian_noise(noise_params_.gyro_bias_stddev)
        );
        accel_bias_ += Vec3(
            gaussian_noise(noise_params_.accel_bias_stddev),
            gaussian_noise(noise_params_.accel_bias_stddev),
            gaussian_noise(noise_params_.accel_bias_stddev)
        );

        // Gyroscope: direct measurement of angular velocity + noise + bias
        Vec3 gyro_noise(
            gaussian_noise(noise_params_.gyro_noise_stddev),
            gaussian_noise(noise_params_.gyro_noise_stddev),
            gaussian_noise(noise_params_.gyro_noise_stddev)
        );
        reading.set_gyro(true_state.angular_velocity + gyro_bias_ + gyro_noise);

        // Accelerometer: specific force in body frame
        // a_meas = R_bn * (a_ned - g_ned) where g_ned = [0, 0, g]
        // In static/hover: accelerometer reads -g in body frame
        
        Mat3 R_bn = true_state.R_bn();
        Vec3 gravity_ned(0.0, 0.0, 9.81);
        
        // Specific force (what accelerometer measures)
        Vec3 specific_force = R_bn * (-gravity_ned);
        
        Vec3 accel_noise(
            gaussian_noise(noise_params_.accel_noise_stddev),
            gaussian_noise(noise_params_.accel_noise_stddev),
            gaussian_noise(noise_params_.accel_noise_stddev)
        );
        reading.set_accel(specific_force + accel_bias_ + accel_noise);

        return reading;
    }

private:
    NoiseParams noise_params_;
    Vec3 gyro_bias_ = Vec3::Zero();
    Vec3 accel_bias_ = Vec3::Zero();
};

/**
 * AHRS sensor model (e.g., BNO055).
 * Outputs attitude quaternion directly with some noise.
 */
class AhrsSensor : public SimSensor<AttitudeReading> {
public:
    struct NoiseParams {
        double attitude_noise_stddev = 0.002;  // [rad] noise on euler angles
        double rate_noise_stddev = 0.001;      // [rad/s] noise on rates
    };

    AhrsSensor(const std::string& name = "ahrs",
               double update_rate_hz = 100.0,
               double latency_s = 0.005)
        : SimSensor<AttitudeReading>(name, update_rate_hz, latency_s)
    {}

    void set_noise_params(const NoiseParams& params) { noise_params_ = params; }

protected:
    AttitudeReading sample(const State& true_state, double /*current_time_s*/) override {
        AttitudeReading reading;

        // Add noise to euler angles then convert back to quaternion
        Vec3 euler = true_state.euler_angles();
        Vec3 euler_noise(
            gaussian_noise(noise_params_.attitude_noise_stddev),
            gaussian_noise(noise_params_.attitude_noise_stddev),
            gaussian_noise(noise_params_.attitude_noise_stddev)
        );
        euler += euler_noise;

        // Convert back to quaternion (ZYX convention)
        reading.attitude = Eigen::AngleAxisd(euler.z(), Vec3::UnitZ())
                         * Eigen::AngleAxisd(euler.y(), Vec3::UnitY())
                         * Eigen::AngleAxisd(euler.x(), Vec3::UnitX());

        // Angular rates with noise
        Vec3 rate_noise(
            gaussian_noise(noise_params_.rate_noise_stddev),
            gaussian_noise(noise_params_.rate_noise_stddev),
            gaussian_noise(noise_params_.rate_noise_stddev)
        );
        reading.angular_velocity = true_state.angular_velocity + rate_noise;

        reading.calibration_status = 3;  // Fully calibrated

        return reading;
    }

private:
    NoiseParams noise_params_;
};

/**
 * GPS sensor model.
 */
class GpsSensor : public SimSensor<GpsReading> {
public:
    struct NoiseParams {
        double position_stddev = 2.5;    // [m] horizontal position noise
        double altitude_stddev = 5.0;    // [m] vertical position noise
        double velocity_stddev = 0.1;    // [m/s] velocity noise
    };

    GpsSensor(const std::string& name = "gps",
              double update_rate_hz = 10.0,
              double latency_s = 0.1)
        : SimSensor<GpsReading>(name, update_rate_hz, latency_s)
    {}

    void set_noise_params(const NoiseParams& params) { noise_params_ = params; }

    // Set origin for local NED to GPS conversion
    void set_origin(double lat_deg, double lon_deg, double alt_m) {
        origin_lla_ = Vec3(lat_deg, lon_deg, alt_m);
    }

protected:
    GpsReading sample(const State& true_state, double /*current_time_s*/) override {
        GpsReading reading;

        // Convert local NED to GPS (simplified, flat earth)
        // 1 degree latitude ≈ 111,139 m
        // 1 degree longitude ≈ 111,139 * cos(lat) m
        double meters_per_deg_lat = 111139.0;
        double meters_per_deg_lon = 111139.0 * std::cos(origin_lla_.x() * M_PI / 180.0);

        Vec3 pos_noise(
            gaussian_noise(noise_params_.position_stddev) / meters_per_deg_lat,
            gaussian_noise(noise_params_.position_stddev) / meters_per_deg_lon,
            gaussian_noise(noise_params_.altitude_stddev)
        );

        Vec3 lla_result;
        lla_result.x() = origin_lla_.x() + true_state.position.x() / meters_per_deg_lat + pos_noise.x();
        lla_result.y() = origin_lla_.y() + true_state.position.y() / meters_per_deg_lon + pos_noise.y();
        lla_result.z() = origin_lla_.z() - true_state.position.z() + pos_noise.z();  // NED z is down
        reading.set_lla(lla_result);

        Vec3 vel_noise(
            gaussian_noise(noise_params_.velocity_stddev),
            gaussian_noise(noise_params_.velocity_stddev),
            gaussian_noise(noise_params_.velocity_stddev)
        );
        reading.set_velocity_ned(true_state.velocity + vel_noise);

        reading.fix_type() = 3;  // 3D fix
        reading.num_satellites() = 10;
        reading.hdop() = 1.0;
        reading.vdop() = 1.5;

        return reading;
    }

private:
    NoiseParams noise_params_;
    Vec3 origin_lla_ = Vec3(52.2053, 0.1218, 10.0);  // Default: Cambridge, UK
};

} // namespace sim
