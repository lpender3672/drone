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

template<typename Scalar>
inline void compute_radius(Scalar lat, Scalar& RM, Scalar& RN) {
    Scalar sin_lat = std::sin(lat);
    Scalar sin2_lat = sin_lat * sin_lat;
    Scalar den = Scalar(1.0) - Scalar(e2) * sin2_lat;

    RM = (Scalar(R0) * (Scalar(1.0) - Scalar(e2))) / std::pow(den, Scalar(1.5));
    RN = Scalar(R0) / std::sqrt(den);
}

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1> gravity_ned(Scalar lat_rad, Scalar height_m)
{
    const Scalar sin_lat = std::sin(lat_rad);
    const Scalar sin2_lat = sin_lat * sin_lat;
    const Scalar sin_2lat = std::sin(Scalar(2.0) * lat_rad);
    const Scalar sin2_2lat = sin_2lat * sin_2lat;

    // Somigliana normal gravity (WGS-84)
    const Scalar g0 =
        Scalar(9.780327) * (Scalar(1.0)
        + Scalar(0.0053024) * sin2_lat
        - Scalar(0.0000058) * sin2_2lat);

    // Free-air correction
    const Scalar g = g0 - Scalar(3.086e-6) * height_m;

    // NED frame: +Z is down
    return Eigen::Matrix<Scalar, 3, 1>(Scalar(0), Scalar(0), g);
}

#endif