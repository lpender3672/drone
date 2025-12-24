#ifndef ES_EKF_H
#define ES_EKF_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <iostream>

#include "ekf_defs.h"

// [Source: 40] Nominal State Dimension: 16
// [Source: 51] Error State Dimension: 15
// [Source: 184] Noise Dimension: 12
const int DIM_NOMINAL = 16;
const int DIM_ERROR = 16;
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

class EKF16d_OPT {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EKF16d_OPT();

    // Initialization [Source: 274]
    void initialize(const Eigen::Vector3d& init_pos, 
                    const Eigen::Vector3d& init_vel, 
                    const Eigen::Quaterniond& init_quat,
                    const Eigen::Vector3d& init_ba,
                    const Eigen::Vector3d& init_bg,
                    double init_bbaro,
                    const Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>& init_P);

    // Prediction Step [Source: 282]
    void predict(const ImuMeasurement& imu, double dt);

    // Measurement Updates [Source: 236]
    void update_gnss_position(const Eigen::Vector3d& pos_gnss, const Eigen::Matrix3d& R);
    void update_gnss_velocity(const Eigen::Vector3d& vel_gnss, const Eigen::Matrix3d& R);
    // [Source: 260] Barometric altitude update
    void update_barometer(double altitude, double R_var);
    void update_magnetometer(const Eigen::Vector3d& mag_body, const Eigen::Matrix3d& R);

    // Getters
    NominalState getState() const { return x_; }
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> getCovariance() const { return P_; }
    EkfStatus getStatus(double max_variance_threshold) const;

    void (*debugCallback)(const char* label) = nullptr;

private:
    // State and Covariance
    NominalState x_;
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> P_;

    // Process Noise Spectral Density [Source: 190]
    Eigen::Matrix<double, DIM_NOISE, DIM_NOISE> Qc_;

    // Constants [Source: 148-150, 140]
    const double R0 = 6378137.0;            // Equatorial Radius
    const double e2 = 0.00669437999014;     // Eccentricity squared
    const double g0 = 9.80665;              // Gravity
    const double WE = 7.292115e-5;          // Earth rotation rate

    Eigen::Vector3d tau_a_;           // Accel bias correlation time [s]
    Eigen::Vector3d tau_g_;           // Gyro bias correlation time [s]
    double tau_bbaro_;               // Baro bias correlation time [s]

    // scratch memory
    struct EkfScratch {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Van Loan discretization (32x32 = 8KB each)
        Eigen::Matrix<double, 2*DIM_ERROR, 2*DIM_ERROR> vl_A;
        Eigen::Matrix<double, 2*DIM_ERROR, 2*DIM_ERROR> vl_B;
        Eigen::Matrix<double, 2*DIM_ERROR, 2*DIM_ERROR> vl_A2;  // A*A term
        
        // Predict matrices (16x16 = 2KB each, 16x13 = 1.6KB)
        Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> F;
        Eigen::Matrix<double, DIM_ERROR, DIM_NOISE> G;
        Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> GQGt;
        Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> Phi;
        Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> Qd;
        Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> Phi_P;  // Phi * P intermediate
        
        // Update matrices - sized for largest update (3 measurements)
        Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> I_KH;
        Eigen::Matrix<double, DIM_ERROR, 1> dx;
        
        // 3-measurement update (GNSS pos/vel, magnetometer)
        Eigen::Matrix<double, 3, 3> S3;
        Eigen::Matrix<double, DIM_ERROR, 3> K3;
        Eigen::Matrix<double, DIM_ERROR, 3> PH3t;
        Eigen::Matrix<double, 3, DIM_ERROR> H3;
        Eigen::Vector3d z3;
        Eigen::Matrix<double, 3, 3> R3;
        
        // 1-measurement update (barometer)
        Eigen::Matrix<double, DIM_ERROR, 1> K1;
        Eigen::Matrix<double, DIM_ERROR, 1> PH1t;
        Eigen::Matrix<double, 1, DIM_ERROR> H1;
        
        // Scratch vectors
        Eigen::Vector3d v3_a, v3_b, v3_c;
        Eigen::Matrix3d C_bn;
    } scratch_;

    // Helpers
    Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    void compute_radius(double lat, double& RM, double& RN);
    
    template<int M>
    void update_internal(
        const Eigen::Matrix<double, M, 1>& z,
        const Eigen::Matrix<double, M, DIM_ERROR>& H,
        const Eigen::Matrix<double, M, M>& R);
    void inject_error(const Eigen::Matrix<double, DIM_ERROR, 1>& dx);

};

#endif // ES_EKF_H