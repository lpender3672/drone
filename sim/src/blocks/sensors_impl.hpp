#pragma once

#include "sim_sensor.hpp"
#include "../data/sensor_reading.hpp"
#include <sensor_constants.h>

namespace sim {
/**
 * IMU sensor model.
 * Outputs accelerometer and gyroscope readings with noise and bias.
 */
template<typename TrueStateT>
class ImuSensor : public SensorBlock<TrueStateT, ImuData> {
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
              uint32_t update_period_us = 1000, 
              uint32_t latency_us = 1000)
        : SensorBlock<TrueStateT, ImuData>(name, update_period_us, latency_us)
    {}

    void set_noise_params(const NoiseParams& params) { noise_params_ = params; }

    void reset() {
        gyro_bias_ = Vec3::Zero();
        accel_bias_ = Vec3::Zero();
    }

protected:
    ImuData sample(const TrueStateT& true_state, uint64_t current_time_us) override {
        ImuData reading;

        // Random walk on biases
        gyro_bias_ += Vec3(
            this->gaussian_noise(noise_params_.gyro_bias_stddev),
            this->gaussian_noise(noise_params_.gyro_bias_stddev),
            this->gaussian_noise(noise_params_.gyro_bias_stddev)
        );
        accel_bias_ += Vec3(
            this->gaussian_noise(noise_params_.accel_bias_stddev),
            this->gaussian_noise(noise_params_.accel_bias_stddev),
            this->gaussian_noise(noise_params_.accel_bias_stddev)
        );

        // Gyroscope: direct measurement of angular velocity + noise + bias
        Vec3 gyro_noise(
            this->gaussian_noise(noise_params_.gyro_noise_stddev),
            this->gaussian_noise(noise_params_.gyro_noise_stddev),
            this->gaussian_noise(noise_params_.gyro_noise_stddev)
        );
        reading.set_gyro(true_state.angular_velocity + gyro_bias_ + gyro_noise);

        // Accelerometer: specific force in body frame
        // a_meas = R_bn * (a_ned - g_ned) where g_ned = [0, 0, g]
        // In static/hover: accelerometer reads -g in body frame
        
        Mat3 R_bn = true_state.R_bn();
        Vec3 gravity_vec(0.0, 0.0, sensors::GRAVITY_MS2);
        
        // Specific force: R_bn * (a_ned - g_ned); g_ned points down in NED
        Vec3 specific_force = R_bn * (true_state.linear_accel - gravity_vec);
        
        Vec3 accel_noise(
            this->gaussian_noise(noise_params_.accel_noise_stddev),
            this->gaussian_noise(noise_params_.accel_noise_stddev),
            this->gaussian_noise(noise_params_.accel_noise_stddev)
        );
        reading.set_acc(specific_force + accel_bias_ + accel_noise);

        (void)current_time_us;  // Unused
        return reading;
    }

private:
    NoiseParams noise_params_;
    Vec3 gyro_bias_ = Vec3::Zero();
    Vec3 accel_bias_ = Vec3::Zero();
};

/**
 * GPS sensor model.
 */
template<typename TrueStateT>
class GpsSensor : public SensorBlock<TrueStateT, GnssData> {
public:
    struct NoiseParams {
        double position_stddev = 2.5;    // [m] horizontal position noise
        double altitude_stddev = 5.0;    // [m] vertical position noise
        double velocity_stddev = 0.1;    // [m/s] velocity noise
    };

    GpsSensor(const std::string& name = "gps",
              uint32_t update_period_us = 100000,
              uint32_t latency_us = 100000)
        : SensorBlock<TrueStateT, GnssData>(name, update_period_us, latency_us)
    {}

    void set_noise_params(const NoiseParams& params) { noise_params_ = params; }

    // Set origin for local NED to GPS conversion
    void set_origin(double lat_deg, double lon_deg, double alt_m) {
        origin_lla_ = Vec3(lat_deg, lon_deg, alt_m);
    }

protected:
    GnssData sample(const TrueStateT& true_state, uint64_t current_time_us) override {
        GnssData reading;

        // Convert local NED to GPS (simplified, flat earth)
        const double lat_rad = origin_lla_.x() * M_PI / 180.0;
        const double meters_per_deg_lat = sensors::METERS_PER_DEG_LAT;
        const double meters_per_deg_lon = sensors::meters_per_deg_lon(lat_rad);

        Vec3 pos_noise(
            this->gaussian_noise(noise_params_.position_stddev) / meters_per_deg_lat,
            this->gaussian_noise(noise_params_.position_stddev) / meters_per_deg_lon,
            this->gaussian_noise(noise_params_.altitude_stddev)
        );

        Vec3 lla_result;
        lla_result.x() = origin_lla_.x() + true_state.position.x() / meters_per_deg_lat + pos_noise.x();
        lla_result.y() = origin_lla_.y() + true_state.position.y() / meters_per_deg_lon + pos_noise.y();
        lla_result.z() = origin_lla_.z() - true_state.position.z() + pos_noise.z();  // NED z is down
        reading.set_lla(lla_result);

        Vec3 vel_noise(
            this->gaussian_noise(noise_params_.velocity_stddev),
            this->gaussian_noise(noise_params_.velocity_stddev),
            this->gaussian_noise(noise_params_.velocity_stddev)
        );
        reading.set_velocity_ned(true_state.velocity + vel_noise);

        reading.set_fix_type(3);  // 3D fix
        reading.set_num_satellites(10);
        reading.set_hdop(1.0);
        reading.set_vdop(1.5);

        (void)current_time_us;  // Unused
        return reading;
    }

private:
    NoiseParams noise_params_;
    Vec3 origin_lla_ = Vec3(52.2053, 0.1218, 10.0);  // Default: Cambridge, UK
};

/**
 * Magnetometer sensor model.
 * Rotates the reference NED field into body frame and adds white noise.
 * Reference vector matches the hardcoded value in EKF16d::update_magnetometer.
 */
template<typename TrueStateT>
class MagSensor : public SensorBlock<TrueStateT, MagData> {
public:
    struct NoiseParams {
        double noise_stddev = 0.5;  // [µT] per axis white noise
    };

    MagSensor(const std::string& name = "mag",
              uint32_t update_period_us = 20000,
              uint32_t latency_us = 5000)
        : SensorBlock<TrueStateT, MagData>(name, update_period_us, latency_us)
    {}

    void set_noise_params(const NoiseParams& params) { noise_params_ = params; }

protected:
    MagData sample(const TrueStateT& true_state, uint64_t current_time_us) override {
        // Reference NED field (unit vector) — matches the EKF's default for Cambridge, UK
        // (WMM 2025: inclination ~67°, declination ~1° W). Override both ends together
        // if you change the sim's GPS origin to another location.
        static const Vec3 m_n(0.391, -0.007, 0.921);
        static const double field_magnitude = 49.0;  // µT (Cambridge, WMM 2025)

        Vec3 m_body = true_state.R_bn() * m_n * field_magnitude;

        Vec3 noise(
            this->gaussian_noise(noise_params_.noise_stddev),
            this->gaussian_noise(noise_params_.noise_stddev),
            this->gaussian_noise(noise_params_.noise_stddev)
        );

        MagData reading;
        reading.set_field(m_body + noise);
        reading.valid = true;

        (void)current_time_us;
        return reading;
    }

private:
    NoiseParams noise_params_;
};

/**
 * Barometer sensor model.
 * Converts true NED z-position to altitude with white noise and bias random walk.
 */
template<typename TrueStateT>
class BaroSensor : public SensorBlock<TrueStateT, BaroData> {
public:
    struct NoiseParams {
        double altitude_noise_stddev = 0.3;   // [m] white noise
        double bias_stddev           = 0.01;  // [m] bias random walk per sample
    };

    BaroSensor(const std::string& name = "baro",
               uint32_t update_period_us = 50000,
               uint32_t latency_us = 50000)
        : SensorBlock<TrueStateT, BaroData>(name, update_period_us, latency_us)
    {}

    void set_noise_params(const NoiseParams& params) { noise_params_ = params; }
    void set_ground_alt_m(double alt_m) { ground_alt_m_ = alt_m; }

protected:
    BaroData sample(const TrueStateT& true_state, uint64_t current_time_us) override {
        // NED z is positive-down; altitude is positive-up relative to ground
        const double true_alt = ground_alt_m_ - true_state.position.z();

        bias_ += this->gaussian_noise(noise_params_.bias_stddev);
        const double meas_alt = true_alt + bias_ + this->gaussian_noise(noise_params_.altitude_noise_stddev);

        BaroData reading;
        reading.altitude_m    = meas_alt;
        reading.pressure_pa   = 101325.0 * std::pow(1.0 - meas_alt / 44330.0, 5.255);
        reading.temperature_c = 15.0 - 0.0065 * meas_alt;
        reading.valid         = true;

        (void)current_time_us;
        return reading;
    }

private:
    NoiseParams noise_params_;
    double ground_alt_m_ = 0.0;
    double bias_         = 0.0;
};

} // namespace sim
