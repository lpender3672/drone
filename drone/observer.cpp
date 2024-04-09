
#include "observer.h"



EKF::EKF(float dt, const Eigen::Vector3f& av_sbsnsk_x, const Eigen::Vector3f& av_sbsnsk_y, const Eigen::Vector3f& av_sbsnsk_z,
         const Eigen::Vector3f& asd_nk_x, const Eigen::Vector3f& asd_nk_y, const Eigen::Vector3f& asd_nk_z)
    : dt_(dt), Q_(Eigen::Matrix<float, 15, 15>::Zero()), R_(Eigen::Matrix<float, 7, 7>::Identity())
{
    // Initialize the process noise covariance matrix (Q)
    Q_.block<3, 3>(6, 6) = av_sbsnsk_x.asDiagonal();
    Q_.block<3, 3>(9, 9) = av_sbsnsk_y.asDiagonal();
    Q_.block<3, 3>(12, 12) = av_sbsnsk_z.asDiagonal();
    
    // Add random walk noise to the process noise covariance matrix (Q)
    Q_.block<3, 3>(3, 3) = (asd_nk_x.array() * dt_).matrix().asDiagonal();
    Q_.block<3, 3>(6, 6) += (asd_nk_y.array() * dt_).matrix().asDiagonal();
    Q_.block<3, 3>(9, 9) += (asd_nk_z.array() * dt_).matrix().asDiagonal();
    
    // Initialize the state transition matrix (F)
    F_.setIdentity();
    
    // Initialize the observation matrix (H)
    H_.setZero();
    H_.bottomLeftCorner<7, 7>().setIdentity();
    
    // Initialize the state estimate (x) and covariance matrix (P)
    x_.setZero();
    P_.setIdentity();
}


void EKF::predict(const Eigen::Vector3f& acc_meas, const Eigen::Quaternionf& q_meas, float current_dt) {
    // Extract orientation quaternion from the state vector
    Eigen::Quaternionf q(x_.segment<4>(6));

    // Update orientation quaternion with the measured quaternion
    q = q_meas;
    x_.segment<4>(6) << q.w(), q.x(), q.y(), q.z();

    // Rotate accelerometer measurements from body frame to navigation frame
    Eigen::Vector3f acc_nav = q * (acc_meas - x_.segment<3>(9));

    // Remove gravity from the accelerometer measurements
    acc_nav -= Eigen::Vector3f(0, 0, 9.81);

    // Integrate accelerometer measurements to update velocity
    x_.segment<3>(3) += acc_nav * current_dt;

    // Integrate velocity to update position
    x_.segment<3>(0) += x_.segment<3>(3) * current_dt;

    // Propagate covariance matrix
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void EKF::update(const Eigen::Vector3f& acc, const Eigen::Quaternionf& q_meas) {
    // Perform the EKF update step

    // Update the observation matrix (H) to include both accelerometer and quaternion measurements
    H_.setZero();
    H_.block<3, 3>(6, 0) = Eigen::Matrix3f::Identity();  // Accelerometer measurement
    H_.block<4, 4>(9, 3) = Eigen::Matrix4f::Identity();  // Quaternion measurement

    // Compute the Kalman gain (K)
    Eigen::Matrix<float, 7, 7> S = H_.transpose() * P_ * H_ + R_;
    Eigen::Matrix<float, 15, 7> K = P_ * H_ * S.inverse();

    // Compute the measurement residual (z - h(x))
    Eigen::Matrix<float, 7, 1> residual;
    residual.block<3, 1>(0, 0) = acc - x_.block<3, 1>(6, 0);
    residual.block<4, 1>(3, 0) = q_meas.coeffs() - x_.block<4, 1>(9, 0);

    // Update the state estimate (x) and covariance matrix (P)
    x_ = x_ + K * residual;
    P_ = (Eigen::Matrix<float, 15, 15>::Identity() - K * H_.transpose()) * P_;
}

Eigen::Matrix<float, 15, 1> EKF::getStateEstimate() {
    return x_;
}
