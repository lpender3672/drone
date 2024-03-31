
#include "observer.h"


Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return m;
}

EKF::EKF(int n, int m, int p) : n_(n), m_(m), p_(p),
    x_(n), P_(n, n), Q_(n, n), R_(m, m), F_(n, n), H_(m, n),
    bias_(n), bias_instability_(n), noise_(n), scale_factor_(6), misalignment_(3, 3),
    acc_white_noise_(0.0), acc_bias_instability_(0.0), acc_rate_random_walk_(0.0),
    gyro_white_noise_(0.0), gyro_bias_instability_(0.0), gyro_rate_random_walk_(0.0)
{
    // Initialize matrices and vectors
    x_.setZero();
    P_.setIdentity();
    Q_.setZero();
    R_.setZero();
    F_.setIdentity();
    H_.setZero();
    bias_.setZero();
    bias_instability_.setZero();
    noise_.setZero();
    scale_factor_.setOnes();
    misalignment_.setIdentity();
}

EKF::~EKF() {}

void EKF::initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
}

void EKF::predict(const Eigen::Vector3d& acc_meas, const Eigen::Quaterniond& q_meas, double dt) {
    // Update error model
    updateErrorModel(dt);

    // Perform prediction step
    x_.head<3>() += x_.segment<3>(3) * dt + 0.5 * acc_meas * dt * dt;  // Update position
    x_.segment<3>(3) += acc_meas * dt;  // Update velocity

    // Update orientation using the quaternion measurement
    Eigen::Quaterniond q_est(x_(6), x_(7), x_(8), x_(9));
    Eigen::Quaterniond q_update = q_meas * q_est.conjugate();
    q_update.normalize();
    x_.segment<4>(6) = (q_est * q_update).coeffs();

    P_ = F_ * P_ * F_.transpose() + Q_;
}

void EKF::update(const Eigen::VectorXd& z) {
    // Update measurement noise covariance
    updateMeasurementNoiseCovariance();

    // Compute Kalman gain
    Eigen::MatrixXd K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();

    // Perform update step
    x_ = x_ + K * (z - H_ * x_);
    P_ = (Eigen::MatrixXd::Identity(n_, n_) - K * H_) * P_;
}

Eigen::VectorXd EKF::getState() const {
    // 3 position and 4 quaternion elements

    Eigen::VectorXd state(7);
    state << x_.head<3>(), x_.segment<4>(6);
    return state;
}

Eigen::MatrixXd EKF::getCovariance() const {
    return P_;
}

void EKF::updateErrorModel(double dt) {
    // Update bias instability
    bias_instability_ = bias_instability_ * std::exp(-dt / bias_instability_time_constant_);

    // Update white noise
    noise_ = noise_ * std::sqrt(dt);

    // Update bias
    bias_ = bias_ + bias_instability_ + noise_;

    // Update state transition matrix
    F_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    F_.block<3, 3>(3, 6) = -skew_symmetric(misalignment_ * ((scale_factor_.segment<3>(0)).asDiagonal() * x_.segment<3>(3))) * dt;
    F_.block<3, 3>(6, 9) = -skew_symmetric(misalignment_ * ((scale_factor_.segment<3>(3)).asDiagonal() * x_.segment<3>(6))) * dt;

    // Update process noise covariance
    updateProcessNoiseCovariance(dt);
}

void EKF::updateProcessNoiseCovariance(double dt) {
    // Update process noise covariance matrix based on error model parameters
    Q_.block<3, 3>(0, 0) = acc_white_noise_ * Eigen::Matrix3d::Identity() * dt;
    Q_.block<3, 3>(3, 3) = gyro_white_noise_ * Eigen::Matrix3d::Identity() * dt;
    Q_.block<3, 3>(6, 6) = acc_bias_instability_ * Eigen::Matrix3d::Identity() * dt;
    Q_.block<3, 3>(9, 9) = gyro_bias_instability_ * Eigen::Matrix3d::Identity() * dt;
}

void EKF::updateMeasurementNoiseCovariance() {
    // Update measurement noise covariance matrix based on Allan variance parameters
    R_.block<3, 3>(0, 0) = acc_white_noise_ * Eigen::Matrix3d::Identity();
    R_.block<3, 3>(3, 3) = gyro_white_noise_ * Eigen::Matrix3d::Identity();
}
