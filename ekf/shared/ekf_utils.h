#ifndef EKF_UTILS_H
#define EKF_UTILS_H

#include <cmath>
#include <Eigen/Dense>

constexpr double R0 = 6378137.0;            // Equatorial Radius
constexpr double e2 = 0.00669437999014;     // Eccentricity squared

template<typename Scalar>
class BufferReserverT {
    Scalar* base_;
    int capacity_;
    int offset_ = 0;
public:
    BufferReserverT(Scalar* buf, int capacity) : base_(buf), capacity_(capacity) {}

    template<int Rows, int Cols>
    Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols>> matrix() {
        static_assert(Rows > 0 && Cols > 0, "Dynamic size not supported");
        int size = Rows * Cols;

        if (offset_ + size > capacity_)
        {
            while (true); // Buffer overflow
        }

        auto map = Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols>>(base_ + offset_);
        offset_ += size;
        return map;
    }

    template<int Size>
    Eigen::Map<Eigen::Matrix<Scalar, Size, 1>> vector() {
        static_assert(Size > 0, "Dynamic size not supported");

        if (offset_ + Size > capacity_)
        {
            while (true); // Buffer overflow
        }

        auto map = Eigen::Map<Eigen::Matrix<Scalar, Size, 1>>(base_ + offset_);
        offset_ += Size;
        return map;
    }

    void reset() { offset_ = 0; }
    int used() const { return offset_; }
    int remaining() const { return capacity_ - offset_; }
};

using BufferReserver = BufferReserverT<double>;

inline void compute_radius(double lat, double& RM, double& RN) {
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