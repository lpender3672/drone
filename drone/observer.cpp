
#include "observer.h"



EKF::EKF(float dt, const Eigen::Vector3f& av_sbsnsk, const Eigen::Vector3f& asd_nk)
    : dt_(dt), Q_(Eigen::Matrix<float, 7, 7>::Zero())
{

    float T = 1/60;

    Eigen::Matrix2f Q_zd;
    Q_zd << av_sbsnsk[1] * T, 0,
            0, av_sbsnsk[2] * T;

    float Q_zd;
    Q_etad << av_sbsnsk[0] / T;

    // process noise covariance matrix (Q)
    Q.block<2, 2>(0, 0) = Q_zd;
    Q(2, 2) = Q_etad;
    
    F_.setIdentity();
    
    H_.setZero();
    H_.bottomLeftCorner<7, 7>().setIdentity();
    
    x_.setZero();
    P_.setIdentity();
}


void EKF::predict() {

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void EKF::update(const float z) {

    Eigen::VectorXd z_pred = H_ * x_;
    Eigen::VectorXd y = z - z_pred;

    Eigen::MatrixXd S = H * P * H.transpose() + R;

    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    
    x_ = x_ + K * y;

    P = (Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K * H) * P;
}

Eigen::Matrix<float, 15, 1> EKF::getStateEstimate() {
    return x_;
}
