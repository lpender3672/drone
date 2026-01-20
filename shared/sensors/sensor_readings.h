#ifndef SHARED_SENSOR_READINGS_H
#define SHARED_SENSOR_READINGS_H

#include <cstdint>
#include <Eigen/Dense>

namespace sensors {

using Vec3 = Eigen::Vector3d;

/**
 * IMU measurement: [acc(3), gyro(3), timestamp, valid] = 8 elements
 */
struct ImuMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static constexpr int DataSize = 8;
    
    uint32_t timestamp_us = 0;
    Vec3 acc = Vec3::Zero();
    Vec3 gyro = Vec3::Zero();
    bool valid = false;

    // Pack to flat array for interblock data
    void to_array(double* data) const {
        data[0] = acc.x();
        data[1] = acc.y();
        data[2] = acc.z();
        data[3] = gyro.x();
        data[4] = gyro.y();
        data[5] = gyro.z();
        data[6] = static_cast<double>(timestamp_us);
        data[7] = valid ? 1.0 : 0.0;
    }

    // Unpack from flat array
    void from_array(const double* data) {
        acc.x() = data[0];
        acc.y() = data[1];
        acc.z() = data[2];
        gyro.x() = data[3];
        gyro.y() = data[4];
        gyro.z() = data[5];
        timestamp_us = static_cast<uint32_t>(data[6]);
        valid = data[7] > 0.5;
    }

    void set_acc(const Vec3& a) {
        acc = a;
    }
    void set_gyro(const Vec3& g) {
        gyro = g;
    }
};

/**
 * Magnetometer measurement: [field(3), timestamp, valid] = 5 elements
 */
struct MagMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static constexpr int DataSize = 5;
    
    uint32_t timestamp_us = 0;
    Vec3 field = Vec3::Zero();
    bool valid = false;

    void to_array(double* data) const {
        data[0] = field.x();
        data[1] = field.y();
        data[2] = field.z();
        data[3] = static_cast<double>(timestamp_us);
        data[4] = valid ? 1.0 : 0.0;
    }

    void from_array(const double* data) {
        field.x() = data[0];
        field.y() = data[1];
        field.z() = data[2];
        timestamp_us = static_cast<uint32_t>(data[3]);
        valid = data[4] > 0.5;
    }

    void set_field(const Vec3& f) {
        field = f;
    }
};

/**
 * Barometer measurement: [pressure, temperature, altitude, timestamp, valid] = 5 elements
 */
struct BaroMeasurement {
    static constexpr int DataSize = 5;
    
    uint32_t timestamp_us = 0;
    double pressure_pa = 101325.0;
    double temperature_c = 25.0;
    double altitude_m = 0;
    bool valid = false;

    void to_array(double* data) const {
        data[0] = pressure_pa;
        data[1] = temperature_c;
        data[2] = altitude_m;
        data[3] = static_cast<double>(timestamp_us);
        data[4] = valid ? 1.0 : 0.0;
    }

    void from_array(const double* data) {
        pressure_pa = data[0];
        temperature_c = data[1];
        altitude_m = data[2];
        timestamp_us = static_cast<uint32_t>(data[3]);
        valid = data[4] > 0.5;
    }
};

/**
 * GNSS measurement: [lat, lon, alt, vel(3), fix, sats, hdop, vdop, timestamp, valid] = 12 elements
 */
struct GnssMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static constexpr int DataSize = 12;
    
    uint32_t timestamp_us = 0;
    double latitude_deg = 0;
    double longitude_deg = 0;
    double altitude_m = 0;
    Vec3 velocity_ned = Vec3::Zero();
    uint8_t fix_type = 0;
    uint8_t num_satellites = 0;
    float hdop = 99.9f;
    float vdop = 99.9f;
    bool valid = false;

    void to_array(double* data) const {
        data[0] = latitude_deg;
        data[1] = longitude_deg;
        data[2] = altitude_m;
        data[3] = velocity_ned.x();
        data[4] = velocity_ned.y();
        data[5] = velocity_ned.z();
        data[6] = static_cast<double>(fix_type);
        data[7] = static_cast<double>(num_satellites);
        data[8] = static_cast<double>(hdop);
        data[9] = static_cast<double>(vdop);
        data[10] = static_cast<double>(timestamp_us);
        data[11] = valid ? 1.0 : 0.0;
    }

    void from_array(const double* data) {
        latitude_deg = data[0];
        longitude_deg = data[1];
        altitude_m = data[2];
        velocity_ned.x() = data[3];
        velocity_ned.y() = data[4];
        velocity_ned.z() = data[5];
        fix_type = static_cast<uint8_t>(data[6]);
        num_satellites = static_cast<uint8_t>(data[7]);
        hdop = static_cast<float>(data[8]);
        vdop = static_cast<float>(data[9]);
        timestamp_us = static_cast<uint32_t>(data[10]);
        valid = data[11] > 0.5;
    }

    void set_lla(const Vec3& lla) {
        latitude_deg = lla.x();
        longitude_deg = lla.y();
        altitude_m = lla.z();
    }
    void set_velocity_ned(const Vec3& vel) {
        velocity_ned = vel;
    }
    void set_fix_type(uint8_t fix) {
        fix_type = fix;
    }
    void set_num_satellites(uint8_t sats) {
        num_satellites = sats;
    }
    void set_hdop(float hd) {
        hdop = hd;
    }
    void set_vdop(float vd) {
        vdop = vd;
    }
};

using ImuReading = ImuMeasurement;
using MagReading = MagMeasurement;
using GnssReading = GnssMeasurement;
using BaroReading = BaroMeasurement;

}  // namespace sensors

#endif  // SHARED_SENSOR_READINGS_H