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

class IEKF {
public:
    virtual ~IEKF() = default;

    virtual void update_magnetometer(const Eigen::Vector3d&, const Eigen::Matrix3d&) = 0;
    virtual void update_gnss_position(const Eigen::Vector3d&, const Eigen::Matrix3d&) = 0;
    virtual void update_gnss_velocity(const Eigen::Vector3d&, const Eigen::Matrix3d&) = 0;
    virtual void update_barometer(const double, const double) = 0;

    virtual void predict(const ImuMeasurement&, double) = 0;
};

template<int DIM_NOMINAL, int DIM_ERROR, int DIM_NOISE>
class EKF : public IEKF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Type aliases
    using NominalVector = Eigen::Matrix<double, DIM_NOMINAL, 1>;
    using ErrorVector = Eigen::Matrix<double, DIM_ERROR, 1>;
    using CovMatrix = Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>;
    using NoiseMatrix = Eigen::Matrix<double, DIM_NOISE, DIM_NOISE>;
    using ProcessNoiseMap = Eigen::Matrix<double, DIM_ERROR, DIM_NOISE>;

    template<int M>
    using MeasVector = Eigen::Matrix<double, M, 1>;
    template<int M>
    using MeasMatrix = Eigen::Matrix<double, M, DIM_ERROR>;
    template<int M>
    using MeasCov = Eigen::Matrix<double, M, M>;

    void (*debugCallback)(const char* label) = nullptr;

    explicit EKF(const EkfErrorParameters& p)
    { }

    void initialize(const NominalVector& x0, const CovMatrix& P0) {
        x_ = x0;
        P_ = P0;
    }
    
    NominalVector getState() { return x_; }
    CovMatrix getCovariance() {return P_; }

    EkfStatus getStatus(double max_variance_threshold) const
    {
        EkfStatus status;
        status.positive_definite_guaranteed = true;
        status.symmetry_ok = true;
        status.diagonal_positive = true;
        status.variances_bounded = true;
        status.min_gershgorin_lower = std::numeric_limits<double>::max();
        status.max_variance = 0.0;
        status.suspect_state = -1;

        constexpr double SYM_TOL = 1e-10;

        for (int i = 0; i < DIM_ERROR; i++) {
            double diag = P_(i, i);
            
            // Check diagonal positive
            if (diag <= 0) {
                status.diagonal_positive = false;
                status.suspect_state = i;
            }
            
            // Track max variance
            if (diag > status.max_variance) {
                status.max_variance = diag;
            }
            
            // Check bounded
            if (diag > max_variance_threshold) {
                status.variances_bounded = false;
                if (status.suspect_state < 0) status.suspect_state = i;
            }
            
            // Gershgorin: sum of absolute off-diagonal elements
            double off_diag_sum = 0.0;
            for (int j = 0; j < DIM_ERROR; j++) {
                if (i == j) continue;  
                off_diag_sum += std::abs(P_(i, j));
            
                // Symmetry check
                if (std::abs(P_(i, j) - P_(j, i)) > SYM_TOL) {
                    status.symmetry_ok = false;
                }
            }
            
            // Lower bound on eigenvalue from this disc
            double lower_bound = diag - off_diag_sum;
            if (lower_bound < status.min_gershgorin_lower) {
                status.min_gershgorin_lower = lower_bound;
            }
            
            if (lower_bound <= 0) {
                status.positive_definite_guaranteed = false;
            }
        }
        
        return status;
    }

protected:
    NominalVector x_;
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

        Eigen::Matrix<double, DIM_ERROR, M> K =
            P_ * H.transpose() * S.inverse();

        ErrorVector dx = K * z;

        // Joseph form
        Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> I_KH =
            Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>::Identity()
            - K * H;

        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        inject_error(dx);
    }

    virtual void inject_error(const ErrorVector& dx)
    { }

    Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return m;
    }

};

#endif // EKF_DEFS_H