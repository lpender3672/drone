#ifndef ESKF_H
#define ESKF_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>

// WGS84 Constants
constexpr double WGS84_A = 6378137.0;         
constexpr double WGS84_E2 = 0.00669437999014; 
constexpr double WGS84_G = 9.7803253359;      

struct SensorParams {
    double N_acc, N_gyro;  
    double B_acc, B_gyro;  
    double tau_acc, tau_gyro;
};

class ESKF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ESKF(const SensorParams& params, const Eigen::Vector3d& init_lla) : params_(params) {
        p_ = init_lla; 
        v_.setZero();
        q_.setIdentity(); 
        ba_.setZero();
        bg_.setZero();
        
        // Gravity (updated in predict, initialized to standard)
        g_ << 0, 0, 9.81;

        // --- FIX 1: TIGHTER INITIAL COVARIANCE ---
        // Identity (1.0) is too huge (57 degrees uncertainty). 
        // We set realistic initial uncertainties:
        P_.setZero();
        P_.block<3,3>(0,0).diagonal().fill(1e-6);    // Position: ~1m
        P_.block<3,3>(3,3).diagonal().fill(1e-4);    // Velocity: ~0.01 m/s
        P_.block<3,3>(6,6).diagonal().fill(0.01);    // Attitude: ~5 degrees (0.1 rad)
        P_.block<3,3>(9,9).diagonal().fill(1e-4);    // Accel Bias
        P_.block<3,3>(12,12).diagonal().fill(1e-4);  // Gyro Bias
        
        setupQ();
    }

    // ---------------------------------------------------------
    // PREDICTION (Curved Earth)
    // ---------------------------------------------------------
    void predict(const Eigen::Vector3d& acc_g, const Eigen::Vector3d& gyro_rad, double dt) {
        double lat = p_(0);
        double h = p_(2);
        double sin_lat = sin(lat);
        double cos_lat = cos(lat);
        double tan_lat = tan(lat);
        
        double temp = 1.0 - WGS84_E2 * sin_lat * sin_lat;
        double Rm = WGS84_A * (1.0 - WGS84_E2) / pow(temp, 1.5);
        double Rn = WGS84_A / sqrt(temp);

        // Sensor Compensation
        Eigen::Vector3d f_b = (acc_g * 9.81) - ba_; 
        Eigen::Vector3d w_b = gyro_rad - bg_;       

        // Earth & Transport Rates
        double omega_e = 7.292115e-5; 
        Eigen::Vector3d w_ie_n(omega_e * cos_lat, 0.0, -omega_e * sin_lat);

        Eigen::Vector3d w_en_n;
        w_en_n << v_(1) / (Rn + h),                     
                 -v_(0) / (Rm + h),                     
                 -v_(1) * tan_lat / (Rn + h);           

        Eigen::Matrix3d C_b_n = q_.toRotationMatrix();
        Eigen::Vector3d w_in_b = w_b - C_b_n.transpose() * (w_ie_n + w_en_n);

        // Attitude Update
        Eigen::Vector3d ang_inc = w_in_b * dt;
        double ang_mag = ang_inc.norm();
        if(ang_mag > 1e-12) {
            Eigen::Quaterniond dq(Eigen::AngleAxisd(ang_mag, ang_inc.normalized()));
            q_ = q_ * dq;
            q_.normalize();
        }
        C_b_n = q_.toRotationMatrix();

        // Velocity Update
        // Somigliana Gravity
        double g_L = 9.780327 * (1 + 0.0053024 * sin_lat*sin_lat - 0.0000058 * sin(2*lat)*sin(2*lat));
        g_(2) = g_L - (3.0877e-6 * h); 

        Eigen::Vector3d f_n = C_b_n * f_b;
        Eigen::Vector3d coriolis = (2.0 * w_ie_n + w_en_n).cross(v_);
        Eigen::Vector3d v_dot = f_n - coriolis + g_;
        v_ += v_dot * dt;

        // Position Update
        double lat_dot = v_(0) / (Rm + h);
        double lon_dot = v_(1) / ((Rn + h) * cos_lat);
        double alt_dot = -v_(2);

        p_(0) += lat_dot * dt;
        p_(1) += lon_dot * dt;
        p_(2) += alt_dot * dt;

        updateF(C_b_n, f_n, Rm, Rn, h, lat, dt);
    }

    // ---------------------------------------------------------
    // UPDATES
    // ---------------------------------------------------------
    void updateBaro(double pressure_hpa) {
        double height = 44330.0 * (1.0 - pow(pressure_hpa / 1013.25, 0.1903));
        double y = height - p_(2);
        
        Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
        H(0, 2) = 1.0; 
        double R = 4.0; // Loose variance for Baro (meters^2)
        performUpdate(H, (Eigen::VectorXd(1) << y).finished(), (Eigen::MatrixXd(1,1) << R).finished());
    }

    void updateMag(const Eigen::Vector3d& mag_uT) {
        Eigen::Vector3d euler = q_.toRotationMatrix().eulerAngles(0, 1, 2); 
        double roll = euler[0];
        double pitch = euler[1];
        
        double Xh = mag_uT.x() * cos(pitch) + mag_uT.y() * sin(roll) * sin(pitch) + mag_uT.z() * cos(roll) * sin(pitch);
        double Yh = mag_uT.y() * cos(roll) - mag_uT.z() * sin(roll);
        double meas_yaw = atan2(-Yh, Xh);
        
        double est_yaw = atan2(2.0*(q_.w()*q_.z() + q_.x()*q_.y()), 1.0 - 2.0*(q_.y()*q_.y() + q_.z()*q_.z()));

        double y = meas_yaw - est_yaw;
        if(y > M_PI) y -= 2*M_PI;
        if(y < -M_PI) y += 2*M_PI;

        Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
        H(0, 8) = 1.0; 
        double R = 0.05; // ~13 degrees variance (increase this if mag is noisy)
        performUpdate(H, (Eigen::VectorXd(1) << y).finished(), (Eigen::MatrixXd(1,1) << R).finished());
    }
    
    // Getters
    Eigen::Vector3d getPositionLLA() const { return p_; } 
    Eigen::Vector3d getVelocity() const { return v_; }
    Eigen::Quaterniond getAttitude() const { return q_; }
    Eigen::Vector3d getAccelBias() const { return ba_; }
    Eigen::Vector3d getGyroBias() const { return bg_; }
    Eigen::Matrix<double, 15, 15> getCovariance() const { return P_; }

private:
    SensorParams params_;
    Eigen::Vector3d p_, v_, ba_, bg_, g_;
    Eigen::Quaterniond q_; 
    Eigen::Matrix<double, 15, 15> P_, Q_; 

    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
        return m;
    }

    void setupQ() {
        Q_.setZero();
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * pow(params_.N_acc, 2);
        Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * pow(params_.N_gyro, 2);
        Q_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (2*pow(params_.B_acc, 2)/params_.tau_acc);
        Q_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (2*pow(params_.B_gyro, 2)/params_.tau_gyro);
    }

    void updateF(const Eigen::Matrix3d& C_b_n, const Eigen::Vector3d& f_n, double Rm, double Rn, double h, double lat, double dt) {
        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
        F(0, 3) = 1.0 / (Rm + h) * dt;
        F(1, 4) = 1.0 / ((Rn + h) * cos(lat)) * dt;
        F(2, 5) = -1.0 * dt;
        F.block<3, 3>(3, 6) = -skewSymmetric(f_n) * dt;
        F.block<3, 3>(3, 9) = -C_b_n * dt;
        F.block<3, 3>(6, 12) = -C_b_n * dt;
        F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (1.0 - dt / params_.tau_acc);
        F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (1.0 - dt / params_.tau_gyro);
        P_ = F * P_ * F.transpose() + Q_ * dt;
    }

    // --- FIX 2: CORRECT FRAME INJECTION ---
    void performUpdate(const Eigen::MatrixXd& H, const Eigen::VectorXd& y, const Eigen::MatrixXd& R) {
        Eigen::MatrixXd S = H * P_ * H.transpose() + R;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        Eigen::VectorXd dx = K * y;
        
        p_ += dx.segment<3>(0);
        v_ += dx.segment<3>(3);
        
        // Attitude Update
        Eigen::Vector3d theta_err = dx.segment<3>(6);
        Eigen::Quaterniond dq(1.0, theta_err.x()/2.0, theta_err.y()/2.0, theta_err.z()/2.0);
        dq.normalize();
        
        // CRITICAL FIX: Pre-multiply because errors are in Navigation Frame
        q_ = dq * q_; 
        q_.normalize();
        
        ba_ += dx.segment<3>(9);
        bg_ += dx.segment<3>(12);
        
        P_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P_;
    }
};

#endif