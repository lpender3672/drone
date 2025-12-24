#ifndef EKF_DEFS_H
#define EKF_DEFS_H

#include <Eigen/Dense>


struct EkfStatus {
    bool positive_definite_guaranteed;  // Gershgorin lower bounds all > 0
    bool symmetry_ok;                   // P = P^T within tolerance
    bool diagonal_positive;             // All P(i,i) > 0
    bool variances_bounded;             // No runaway growth
    double min_gershgorin_lower;        // Smallest eigenvalue lower bound
    double max_variance;                // Largest diagonal element
    int suspect_state;                  // Index of most concerning state (-1 if ok)
};

struct ImuMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d acc;    // Specific Force [g]
    Eigen::Vector3d gyro;   // Angular Rate [rad/s]
};

struct EkfErrorParameters {
    double sampling_freq;
    // Gyroscope
    double gyro_x_n, gyro_x_b, gyro_x_tp;
    double gyro_y_n, gyro_y_b, gyro_y_tp;
    double gyro_z_n, gyro_z_b, gyro_z_tp;
    // Accelerometer
    double accel_x_n, accel_x_b, accel_x_tp;
    double accel_y_n, accel_y_b, accel_y_tp;
    double accel_z_n, accel_z_b, accel_z_tp;
    // Barometer
    double baro_altitude_n, baro_altitude_b, baro_altitude_tp;
};

template<int DIM_STATE, int DIM_NOISE>
class EKF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Type aliases
    using StateVector = Eigen::Matrix<double, DIM_STATE, 1>;
    using CovMatrix = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;
    using NoiseMatrix = Eigen::Matrix<double, DIM_NOISE, DIM_NOISE>;
    using ProcessNoiseMap = Eigen::Matrix<double, DIM_STATE, DIM_NOISE>;

    template<int M>
    using MeasVector = Eigen::Matrix<double, M, 1>;
    template<int M>
    using MeasMatrix = Eigen::Matrix<double, M, DIM_STATE>;
    template<int M>
    using MeasCov = Eigen::Matrix<double, M, M>;

    void (*debugCallback)(const char* label) = nullptr;

    explicit EKF(const EkfErrorParameters& p);

    void initialize(const StateVector& x0, const CovMatrix& P0) {
        x_ = x0;
        P_ = P0;
    }
    
    EkfStatus getStatus(double max_variance_threshold) const;
    StateVector getState() { return x_; }
    CovMatrix getCovariance() {return P_; }

protected:
    StateVector x_;
    CovMatrix P_;
    NoiseMatrix Qc_;

    template<int M>
    void update_internal(
        const MeasVector<M>& z,
        const MeasMatrix<M>& H,
        const MeasCov<M>& R)
    {
        Eigen::Matrix<double, M, M> S =
            H * P_ * H.transpose() + R;

        Eigen::Matrix<double, DIM_STATE, M> K =
            P_ * H.transpose() * S.inverse();

        Eigen::Matrix<double, DIM_STATE, 1> dx = K * z;

        // Joseph form
        Eigen::Matrix<double, DIM_STATE, DIM_STATE> I_KH =
            Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity()
            - K * H;

        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        inject_error(dx);
    }

    virtual void inject_error(const Eigen::Matrix<double, DIM_STATE, 1>& dx);

    Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return m;
    }

};

#endif // EKF_DEFS_H