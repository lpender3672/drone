#ifndef EKF_UTILS_H
#define EKF_UTILS_H

#include <cmath>
#include <Eigen/Dense>

constexpr double R0 = 6378137.0;            // Equatorial Radius
constexpr double e2 = 0.00669437999014;     // Eccentricity squared

void compute_radius(double lat, double& RM, double& RN) {
    double sin_lat = sin(lat);
    double sin2_lat = sin_lat * sin_lat;
    double den = 1.0 - e2 * sin2_lat;
    
    RM = (R0 * (1.0 - e2)) / pow(den, 1.5);
    RN = R0 / sqrt(den);
}

inline Eigen::Vector3d gravity_ned(double lat_rad, double height_m)
{
    const double sin_lat = std::sin(lat_rad);
    const double sin2_lat = sin_lat * sin_lat;
    const double sin_2lat = std::sin(2.0 * lat_rad);
    const double sin2_2lat = sin_2lat * sin_2lat;

    // Somigliana normal gravity (WGS-84)
    const double g0 =
        9.780327 * (1.0
        + 0.0053024 * sin2_lat
        - 0.0000058 * sin2_2lat);

    // Free-air correction
    const double g = g0 - 3.086e-6 * height_m;

    // NED frame: +Z is down
    return Eigen::Vector3d(0.0, 0.0, g);
}

#endif