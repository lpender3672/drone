#pragma once

#include "../core/inter_block_data.hpp"
#include "../../../shared/sensors/sensor_readings.h"

namespace sim {

/**
 * Base class for all sensor readings.
 * Each sensor type derives from this.
 */
struct SensorReading : public InterBlockData {
    bool valid = true;  // False if sensor has failed/is unavailable

    SensorReading() = default;
    explicit SensorReading(double timestamp_s) : InterBlockData(timestamp_s) {}

    std::string type_name() const override { return "SensorReading"; }
};

/**
 * IMU reading: accelerometer + gyroscope
 * Wraps the shared sensors::ImuMeasurement
 */
struct ImuReading : public SensorReading {
    sensors::ImuMeasurement data;

    ImuReading() = default;
    explicit ImuReading(double timestamp_s) : SensorReading(timestamp_s) {
        data.timestamp_us = static_cast<uint32_t>(timestamp_s * 1e6);
    }

    std::string type_name() const override { return "ImuReading"; }
    
    InterBlockData* clone() const override {
        return new ImuReading(*this);
    }
    
    // Convenience accessors
    const Vec3& accel() const { return data.acc; }
    const Vec3& gyro() const { return data.gyro; }
    
    void set_accel(const Vec3& a) { data.acc = a; }
    void set_gyro(const Vec3& g) { data.gyro = g; }
};

/**
 * Attitude reading from an AHRS (e.g., BNO055)
 * This is a "processed" sensor that outputs orientation directly
 */
struct AttitudeReading : public SensorReading {
    // Quaternion
    Quat attitude = Quat::Identity();

    // Angular velocity [rad/s] in body frame
    Vec3 angular_velocity = Vec3::Zero();
    
    // Calibration status (0-3 for BNO055)
    int calibration_status = 0;

    AttitudeReading() = default;
    explicit AttitudeReading(double timestamp_s) : SensorReading(timestamp_s) {}

    std::string type_name() const override { return "AttitudeReading"; }
    
    InterBlockData* clone() const override {
        return new AttitudeReading(*this);
    }
};

/**
 * GPS reading
 * Wraps the shared sensors::GnssMeasurement
 */
struct GpsReading : public SensorReading {
    sensors::GnssMeasurement data;

    GpsReading() = default;
    explicit GpsReading(double timestamp_s) : SensorReading(timestamp_s) {
        data.timestamp_us = static_cast<uint32_t>(timestamp_s * 1e6);
    }

    std::string type_name() const override { return "GpsReading"; }
    
    InterBlockData* clone() const override {
        return new GpsReading(*this);
    }

    // Convenience accessors
    Vec3 lla() const { return Vec3(data.latitude_deg, data.longitude_deg, data.altitude_m); }
    const Vec3& velocity_ned() const { return data.velocity_ned; }
    double latitude() const { return data.latitude_deg; }
    double longitude() const { return data.longitude_deg; }
    double altitude() const { return data.altitude_m; }
    
    void set_lla(const Vec3& lla_vec) {
        data.latitude_deg = lla_vec.x();
        data.longitude_deg = lla_vec.y();
        data.altitude_m = lla_vec.z();
    }
    void set_velocity_ned(const Vec3& vel) { data.velocity_ned = vel; }
    
    // Direct access to quality indicators
    uint8_t& fix_type() { return data.fix_type; }
    uint8_t fix_type() const { return data.fix_type; }
    uint8_t& num_satellites() { return data.num_satellites; }
    uint8_t num_satellites() const { return data.num_satellites; }
    float& hdop() { return data.hdop; }
    float hdop() const { return data.hdop; }
    float& vdop() { return data.vdop; }
    float vdop() const { return data.vdop; }
};

/**
 * Barometer reading
 * Wraps the shared sensors::BaroMeasurement
 */
struct BaroReading : public SensorReading {
    sensors::BaroMeasurement data;

    BaroReading() = default;
    explicit BaroReading(double timestamp_s) : SensorReading(timestamp_s) {
        data.timestamp_us = static_cast<uint32_t>(timestamp_s * 1e6);
    }

    std::string type_name() const override { return "BaroReading"; }
    
    InterBlockData* clone() const override {
        return new BaroReading(*this);
    }
    
    // Convenience accessors
    double pressure_pa() const { return data.pressure_pa; }
    double temperature_c() const { return data.temperature_c; }
    double altitude_m() const { return data.altitude_m; }
    
    void set_pressure_pa(double p) { data.pressure_pa = p; }
    void set_temperature_c(double t) { data.temperature_c = t; }
    void set_altitude_m(double a) { data.altitude_m = a; }
};

/**
 * Magnetometer reading
 * Wraps the shared sensors::MagMeasurement
 */
struct MagReading : public SensorReading {
    sensors::MagMeasurement data;

    MagReading() = default;
    explicit MagReading(double timestamp_s) : SensorReading(timestamp_s) {
        data.timestamp_us = static_cast<uint32_t>(timestamp_s * 1e6);
    }

    std::string type_name() const override { return "MagReading"; }
    
    InterBlockData* clone() const override {
        return new MagReading(*this);
    }
    
    // Convenience accessors
    const Vec3& field() const { return data.field; }
    void set_field(const Vec3& f) { data.field = f; }
};

} // namespace sim
