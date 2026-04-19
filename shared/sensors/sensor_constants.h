#ifndef SHARED_SENSOR_CONSTANTS_H
#define SHARED_SENSOR_CONSTANTS_H

#include <cmath>

namespace sensors {

static constexpr double GRAVITY_MS2 = 9.80665;

// Approximate meters per degree of latitude (WGS84 mean)
static constexpr double METERS_PER_DEG_LAT = 111319.9;

// Meters per degree of longitude at a given latitude (radians)
inline double meters_per_deg_lon(double lat_rad) {
    return METERS_PER_DEG_LAT * std::cos(lat_rad);
}

}  // namespace sensors

#endif  // SHARED_SENSOR_CONSTANTS_H
