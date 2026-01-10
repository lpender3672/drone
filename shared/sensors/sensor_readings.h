#ifndef SHARED_SENSOR_READINGS_H
#define SHARED_SENSOR_READINGS_H

#include <Eigen/Dense>
#include <cstdint>

namespace sensors {

// Common Eigen typedefs
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/**
 * Standardized IMU measurement structure.
 * Used across all platforms (sim, teensy, ekf).
 */
struct ImuMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    uint32_t timestamp_us = 0;  // Timestamp in microseconds
    Vec3 acc = Vec3::Zero();    // Specific force [m/s^2] in body frame
    Vec3 gyro = Vec3::Zero();   // Angular rate [rad/s] in body frame
    bool valid = false;
};

/**
 * Standardized magnetometer measurement structure.
 * Used across all platforms.
 */
struct MagMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    uint32_t timestamp_us = 0;  // Timestamp in microseconds
    Vec3 field = Vec3::Zero();  // Magnetic field [uT] in body frame
    bool valid = false;
};

/**
 * Standardized GNSS measurement structure.
 * Used across all platforms.
 */
struct GnssMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    uint32_t timestamp_us = 0;  // Timestamp in microseconds
    
    // Position in LLA
    double latitude_deg = 0;    // [degrees]
    double longitude_deg = 0;   // [degrees]
    double altitude_m = 0;      // [meters] above MSL
    
    // Velocity in NED frame [m/s]
    Vec3 velocity_ned = Vec3::Zero();
    
    // Quality indicators
    uint8_t fix_type = 0;       // 0=no fix, 2=2D, 3=3D
    uint8_t num_satellites = 0;
    float hdop = 99.9f;         // Horizontal dilution of precision
    float vdop = 99.9f;         // Vertical dilution of precision
    
    bool valid = false;
};

/**
 * Standardized barometer measurement structure.
 * Used across all platforms.
 */
struct BaroMeasurement {
    uint32_t timestamp_us = 0;      // Timestamp in microseconds
    double pressure_pa = 101325.0;  // [Pa]
    double temperature_c = 25.0;    // [°C]
    double altitude_m = 0;          // [meters] derived from pressure
    bool valid = false;
};

// Type aliases for backward compatibility
using ImuReading = ImuMeasurement;
using MagReading = MagMeasurement;
using GnssReading = GnssMeasurement;
using BaroReading = BaroMeasurement;

}  // namespace sensors

#endif  // SHARED_SENSOR_READINGS_H
