#pragma once

#include "sensor_readings.h"
#include <Eigen/Dense>

namespace sensors {

// Eigen typedefs for convenience
using Vec3 = Eigen::Vector3d;

/**
 * C++ wrapper for ImuReading with Eigen support
 */
struct ImuReadingCpp : public ImuReading {
    ImuReadingCpp() : ImuReading() {}
    
    // Construct from base struct
    explicit ImuReadingCpp(const ImuReading& base) : ImuReading(base) {}
    
    // Eigen accessors
    Vec3 accel() const { return Vec3(accel_x, accel_y, accel_z); }
    Vec3 gyro() const { return Vec3(gyro_x, gyro_y, gyro_z); }
    
    void set_accel(const Vec3& a) {
        accel_x = static_cast<float>(a.x());
        accel_y = static_cast<float>(a.y());
        accel_z = static_cast<float>(a.z());
    }
    
    void set_gyro(const Vec3& g) {
        gyro_x = static_cast<float>(g.x());
        gyro_y = static_cast<float>(g.y());
        gyro_z = static_cast<float>(g.z());
    }
};

/**
 * C++ wrapper for MagReading with Eigen support
 */
struct MagReadingCpp : public MagReading {
    MagReadingCpp() : MagReading() {}
    
    // Construct from base struct
    explicit MagReadingCpp(const MagReading& base) : MagReading(base) {}
    
    // Eigen accessor
    Vec3 field() const { return Vec3(mag_x, mag_y, mag_z); }
    
    void set_field(const Vec3& f) {
        mag_x = static_cast<float>(f.x());
        mag_y = static_cast<float>(f.y());
        mag_z = static_cast<float>(f.z());
    }
};

/**
 * C++ wrapper for GnssReading with Eigen support
 */
struct GnssReadingCpp : public GnssReading {
    GnssReadingCpp() : GnssReading() {}
    
    // Construct from base struct
    explicit GnssReadingCpp(const GnssReading& base) : GnssReading(base) {}
    
    // Eigen accessors
    Vec3 lla() const { return Vec3(latitude_deg, longitude_deg, altitude_m); }
    Vec3 velocity_ned() const { return Vec3(vel_north, vel_east, vel_down); }
    
    void set_lla(const Vec3& lla_vec) {
        latitude_deg = lla_vec.x();
        longitude_deg = lla_vec.y();
        altitude_m = lla_vec.z();
    }
    
    void set_velocity_ned(const Vec3& vel) {
        vel_north = static_cast<float>(vel.x());
        vel_east = static_cast<float>(vel.y());
        vel_down = static_cast<float>(vel.z());
    }
};

/**
 * C++ wrapper for BaroReading (no Eigen needed but for consistency)
 */
struct BaroReadingCpp : public BaroReading {
    BaroReadingCpp() : BaroReading() {}
    
    // Construct from base struct
    explicit BaroReadingCpp(const BaroReading& base) : BaroReading(base) {}
};

}  // namespace sensors
