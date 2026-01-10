#ifndef SHARED_SENSOR_READINGS_H
#define SHARED_SENSOR_READINGS_H

#include <stdint.h>

namespace sensors {

/**
 * Standardized IMU reading structure.
 * Used across all platforms (sim, teensy, drone).
 */
struct ImuReading {
    uint32_t timestamp_us;  // Timestamp in microseconds
    
    // Accelerometer [m/s^2] in body frame
    float accel_x;
    float accel_y;
    float accel_z;
    
    // Gyroscope [rad/s] in body frame
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    bool valid;  // True if reading is valid
    
    ImuReading() 
        : timestamp_us(0), accel_x(0), accel_y(0), accel_z(0),
          gyro_x(0), gyro_y(0), gyro_z(0), valid(false) {}
};

/**
 * Standardized magnetometer reading structure.
 * Used across all platforms.
 */
struct MagReading {
    uint32_t timestamp_us;  // Timestamp in microseconds
    
    // Magnetic field [uT] in body frame
    float mag_x;
    float mag_y;
    float mag_z;
    
    bool valid;  // True if reading is valid
    
    MagReading()
        : timestamp_us(0), mag_x(0), mag_y(0), mag_z(0), valid(false) {}
};

/**
 * Standardized GNSS reading structure.
 * Used across all platforms.
 */
struct GnssReading {
    uint32_t timestamp_us;  // Timestamp in microseconds
    
    // Position in LLA
    double latitude_deg;   // [degrees]
    double longitude_deg;  // [degrees]
    double altitude_m;     // [meters] above MSL
    
    // Velocity in NED frame [m/s]
    float vel_north;
    float vel_east;
    float vel_down;
    
    // Quality indicators
    uint8_t fix_type;        // 0=no fix, 2=2D, 3=3D
    uint8_t num_satellites;
    float hdop;              // Horizontal dilution of precision
    float vdop;              // Vertical dilution of precision
    
    bool valid;  // True if reading is valid
    
    GnssReading()
        : timestamp_us(0), latitude_deg(0), longitude_deg(0), altitude_m(0),
          vel_north(0), vel_east(0), vel_down(0), fix_type(0), num_satellites(0),
          hdop(99.9f), vdop(99.9f), valid(false) {}
};

/**
 * Standardized barometer reading structure.
 * Used across all platforms.
 */
struct BaroReading {
    uint32_t timestamp_us;  // Timestamp in microseconds
    
    float pressure_pa;      // [Pa]
    float temperature_c;    // [°C]
    float altitude_m;       // [meters] derived from pressure
    
    bool valid;  // True if reading is valid
    
    BaroReading()
        : timestamp_us(0), pressure_pa(101325.0f), temperature_c(25.0f),
          altitude_m(0), valid(false) {}
};

}  // namespace sensors

#endif  // SHARED_SENSOR_READINGS_H
