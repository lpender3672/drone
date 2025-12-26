#ifndef EKF16D_H
#define EKF16D_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <iostream>

#include "ekf.h"

// [Source: 40] Nominal State Dimension: 16
// [Source: 51] Error State Dimension: 15
// [Source: 184] Noise Dimension: 12
const int DIM_STATE = 16;
const int DIM_NOISE = 13;

// Indices for Error State Vector [Source: 44]
enum ErrorIdx {
    IDX_POS = 0,
    IDX_VEL = 3,
    IDX_ATT = 6,
    IDX_BA  = 9,
    IDX_BG  = 12,
    IDX_BBARO = 15
};

struct NominalState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // [Source: 40]
    Eigen::Vector3d p;      // Geodetic Position (lat, lon, h) [rad, rad, m]
    Eigen::Vector3d v;      // Velocity (NED) [m/s]
    Eigen::Quaterniond q;   // Attitude (Body to Nav) [Hamilton, scalar-last]
    Eigen::Vector3d ba;     // Accel Bias [m/s^2]
    Eigen::Vector3d bg;     // Gyro Bias [rad/s]
    double bbaro;           // Barometric Altitude Bias [m]

    NominalState() {
        p.setZero();
        v.setZero();
        q.setIdentity(); // q = [0,0,0,1] in Eigen coefficients (x,y,z,w)
        ba.setZero();
        bg.setZero();
        bbaro = 0.0;
    }
};

class EKF16d : public EKF<DIM_STATE, DIM_NOISE> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EKF16d(const EkfErrorParameters& p);

    // Prediction Step [Source: 282]
    void predict(const ImuMeasurement& imu, double dt);

    // Measurement Updates [Source: 236]
    void update_gnss_position(const Eigen::Vector3d& pos_gnss, const Eigen::Matrix3d& R);
    void update_gnss_velocity(const Eigen::Vector3d& vel_gnss, const Eigen::Matrix3d& R);
    // [Source: 260] Barometric altitude update
    void update_barometer(double altitude, double R_var);
    void update_magnetometer(const Eigen::Vector3d& mag_body, const Eigen::Matrix3d& R);

    void (*debugCallback)(const char* label) = nullptr;

    auto pos()       { return x_.segment<3>(IDX_POS); }
    auto vel()       { return x_.segment<3>(IDX_VEL); }
    auto ba()        { return x_.segment<3>(IDX_BA); }
    auto bg()        { return x_.segment<3>(IDX_BG); }
    double& bbaro()  { return x_(IDX_BBARO); }
    Eigen::Quaterniond& q() { return q_; }

private:
    Eigen::Quaterniond q_;

    // Constants [Source: 148-150, 140]
    const double R0 = 6378137.0;            // Equatorial Radius
    const double e2 = 0.00669437999014;     // Eccentricity squared
    const double g0 = 9.80665;              // Gravity
    const double WE = 7.292115e-5;          // Earth rotation rate

    Eigen::Vector3d tau_a_;           // Accel bias correlation time [s]
    Eigen::Vector3d tau_g_;           // Gyro bias correlation time [s]
    double tau_bbaro_;               // Baro bias correlation time [s]

    // Helpers
    void compute_radius(double lat, double& RM, double& RN);

    void inject_error(const Eigen::Matrix<double, DIM_STATE, 1>& dx);
};

#endif // ES_EKF_H