#include "ekf16d_opt.h"
#include "tuned_imu_params.h"

using namespace IMUErrorModel;

EKF16d_OPT::EKF16d_OPT() {
    P_.setIdentity();
    Qc_.setZero();

    // Load gyro parameters (rad/s units)
    Eigen::Vector3d N_gyro(GYRO_X_N, GYRO_Y_N, GYRO_Z_N);
    Eigen::Vector3d B_gyro(GYRO_X_B, GYRO_Y_B, GYRO_Z_B);
    Eigen::Vector3d Tp_gyro(GYRO_X_TP, GYRO_Y_TP, GYRO_Z_TP);

    // Load accel parameters (m/s² units)
    Eigen::Vector3d N_acc(ACCEL_X_N, ACCEL_Y_N, ACCEL_Z_N);
    Eigen::Vector3d B_acc(ACCEL_X_B, ACCEL_Y_B, ACCEL_Z_B);
    Eigen::Vector3d Tp_acc(ACCEL_X_TP, ACCEL_Y_TP, ACCEL_Z_TP);

    // Load baro parameters (m units)
    double N_baro = BARO_ALTITUDE_N;
    double B_baro = BARO_ALTITUDE_B;
    double Tp_baro = 1e6; // Large time constant to approximate white noise

    // Gauss-Markov time constants
    tau_g_ = Tp_gyro / 1.89;
    tau_a_ = Tp_acc / 1.89;
    tau_bbaro_ = Tp_baro / 1.89;

    // Compute driving noise variances
    Eigen::Vector3d sigma_ba = B_acc.array().square() / (tau_a_.array() * 0.4365 * 0.4365);
    Eigen::Vector3d sigma_bg = B_gyro.array().square() / (tau_g_.array() * 0.4365 * 0.4365);
    double sigma_bbaro = (B_baro * B_baro) / (tau_bbaro_ * 0.4365 * 0.4365);

    Eigen::Vector3d q_ba = 2.0 * sigma_ba.array().square() / tau_a_.array();
    Eigen::Vector3d q_bg = 2.0 * sigma_bg.array().square() / tau_g_.array();
    double q_bbaro = 2.0 * sigma_bbaro * sigma_bbaro / tau_bbaro_;

    Qc_.diagonal() << 
        N_gyro.array().square(),   // 0-2: gyro noise
        N_acc.array().square(),    // 3-5: accel noise
        q_ba,                      // 6-8: accel bias driving noise
        q_bg,                      // 9-11: gyro bias driving noise
        q_bbaro;                   // 12: baro bias driving noise
}

void EKF16d_OPT::initialize(const Eigen::Vector3d& init_pos, 
                       const Eigen::Vector3d& init_vel, 
                       const Eigen::Quaterniond& init_quat,
                       const Eigen::Vector3d& init_ba,
                       const Eigen::Vector3d& init_bg,
                       double init_bbaro,
                       const Eigen::Matrix<double, DIM_STATE, DIM_STATE>& init_P) {
    x_.p = init_pos;
    x_.v = init_vel;
    x_.q = init_quat;
    x_.q.normalize();
    x_.ba = init_ba;
    x_.bg = init_bg;
    x_.bbaro = init_bbaro;
    P_ = init_P;
}

Eigen::Matrix3d EKF16d_OPT::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

void EKF16d_OPT::compute_radius(double lat, double& RM, double& RN) {
    double sin_lat = sin(lat);
    double sin2_lat = sin_lat * sin_lat;
    double den = 1.0 - e2 * sin2_lat;
    
    RM = (R0 * (1.0 - e2)) / pow(den, 1.5);
    RN = R0 / sqrt(den);
}

void EKF16d_OPT::predict(const ImuMeasurement& imu, double dt) {
    // Reuse scratch vectors
    Eigen::Vector3d& f_b = scratch_.v3_a;
    Eigen::Vector3d& w_b = scratch_.v3_b;
    
    f_b = imu.acc - x_.ba;
    w_b = imu.gyro - x_.bg;

    Eigen::Matrix3d& C_b_n = scratch_.C_bn;
    C_b_n = x_.q.toRotationMatrix();
    
    double lat = x_.p.x();
    double h = x_.p.z();
    
    double RM, RN;
    compute_radius(lat, RM, RN);

    // Earth rate in nav frame (inline, no allocation)
    double w_ie_n_x = WE * cos(lat);
    double w_ie_n_z = -WE * sin(lat);

    // Transport rate components
    double w_en_n_x = x_.v.y() / (RN + h);
    double w_en_n_y = -x_.v.x() / (RM + h);
    double w_en_n_z = -x_.v.x() * x_.v.y() * tan(lat) / (RN + h);

    double w_in_n_x = w_ie_n_x + w_en_n_x;
    double w_in_n_y = w_en_n_y;
    double w_in_n_z = w_ie_n_z + w_en_n_z;
    
    // Gravity (Somigliana)
    double sin_lat = sin(lat);
    double sin_2lat = sin(2*lat);
    double g_z = 9.780327 * (1 + 0.0053024*sin_lat*sin_lat - 0.0000058*sin_2lat*sin_2lat) - 3.086e-6*h;

    // Velocity mechanization
    Eigen::Vector3d& f_n = scratch_.v3_c;
    f_n.noalias() = C_b_n * f_b;
    
    // Coriolis term: (2*w_ie_n + w_en_n) x v
    double wx = 2*w_ie_n_x + w_en_n_x;
    double wy = w_en_n_y;
    double wz = 2*w_ie_n_z + w_en_n_z;
    
    double v_dot_x = f_n.x() - (wy*x_.v.z() - wz*x_.v.y());
    double v_dot_y = f_n.y() - (wz*x_.v.x() - wx*x_.v.z());
    double v_dot_z = f_n.z() + g_z - (wx*x_.v.y() - wy*x_.v.x());

    // Position mechanization
    x_.p.x() += x_.v.x() / (RM + h) * dt;
    x_.p.y() += x_.v.y() / ((RN + h) * cos(lat)) * dt;
    x_.p.z() -= x_.v.z() * dt;
    
    x_.v.x() += v_dot_x * dt;
    x_.v.y() += v_dot_y * dt;
    x_.v.z() += v_dot_z * dt;

    // Attitude mechanization
    Eigen::Vector3d w_in_n_vec(w_in_n_x, w_in_n_y, w_in_n_z);
    scratch_.v3_a.noalias() = C_b_n.transpose() * w_in_n_vec;
    Eigen::Vector3d w_nb_b = w_b - scratch_.v3_a;
    
    double angle_x = w_nb_b.x() * dt;
    double angle_y = w_nb_b.y() * dt;
    double angle_z = w_nb_b.z() * dt;
    double angle_norm = sqrt(angle_x*angle_x + angle_y*angle_y + angle_z*angle_z);
    
    Eigen::Quaterniond dq;
    if (angle_norm > 1e-8) {
        double half_angle = 0.5 * angle_norm;
        double s = sin(half_angle) / angle_norm;
        dq = Eigen::Quaterniond(cos(half_angle), s*angle_x, s*angle_y, s*angle_z);
    } else {
        dq = Eigen::Quaterniond(1, 0.5*angle_x, 0.5*angle_y, 0.5*angle_z);
        dq.normalize();
    }
    x_.q = x_.q * dq;
    x_.q.normalize();

    // --- Build F Matrix (use scratch) ---
    auto& F = scratch_.F;
    F.setZero();

    // F_rv (Pos-Vel)
    F(IDX_POS, IDX_VEL)     = 1.0 / (RM + h);
    F(IDX_POS+1, IDX_VEL+1) = 1.0 / ((RN + h) * cos(lat));
    F(IDX_POS+2, IDX_VEL+2) = -1.0;

    // F_vv (Vel-Vel) - Coriolis: -skew(2*w_ie_n + w_en_n)
    F(IDX_VEL,   IDX_VEL+1) =  wz;
    F(IDX_VEL,   IDX_VEL+2) = -wy;
    F(IDX_VEL+1, IDX_VEL)   = -wz;
    F(IDX_VEL+1, IDX_VEL+2) =  wx;
    F(IDX_VEL+2, IDX_VEL)   =  wy;
    F(IDX_VEL+2, IDX_VEL+1) = -wx;

    // F_v_theta (Vel-Att): -skew(f_n)
    F(IDX_VEL,   IDX_ATT+1) =  f_n.z();
    F(IDX_VEL,   IDX_ATT+2) = -f_n.y();
    F(IDX_VEL+1, IDX_ATT)   = -f_n.z();
    F(IDX_VEL+1, IDX_ATT+2) =  f_n.x();
    F(IDX_VEL+2, IDX_ATT)   =  f_n.y();
    F(IDX_VEL+2, IDX_ATT+1) = -f_n.x();

    // F_v_ba (Vel-AccelBias): -C_b_n
    F.block<3,3>(IDX_VEL, IDX_BA) = -C_b_n;

    // F_theta_theta (Att-Att): -skew(w_in_n)
    F(IDX_ATT,   IDX_ATT+1) =  w_in_n_z;
    F(IDX_ATT,   IDX_ATT+2) = -w_in_n_y;
    F(IDX_ATT+1, IDX_ATT)   = -w_in_n_z;
    F(IDX_ATT+1, IDX_ATT+2) =  w_in_n_x;
    F(IDX_ATT+2, IDX_ATT)   =  w_in_n_y;
    F(IDX_ATT+2, IDX_ATT+1) = -w_in_n_x;

    // F_theta_bg (Att-GyroBias): -C_b_n
    F.block<3,3>(IDX_ATT, IDX_BG) = -C_b_n;

    // Gauss-Markov bias dynamics
    F(IDX_BA,   IDX_BA)   = -1.0 / tau_a_.x();
    F(IDX_BA+1, IDX_BA+1) = -1.0 / tau_a_.y();
    F(IDX_BA+2, IDX_BA+2) = -1.0 / tau_a_.z();
    F(IDX_BG,   IDX_BG)   = -1.0 / tau_g_.x();
    F(IDX_BG+1, IDX_BG+1) = -1.0 / tau_g_.y();
    F(IDX_BG+2, IDX_BG+2) = -1.0 / tau_g_.z();
    F(IDX_BBARO, IDX_BBARO) = -1.0 / tau_bbaro_;

    // --- Build G Matrix (use scratch) ---
    auto& G = scratch_.G;
    G.setZero();
    G.block<3,3>(IDX_VEL, 0) = -C_b_n;
    G.block<3,3>(IDX_ATT, 3) = -C_b_n;
    G(IDX_BA,   6) = 1.0;
    G(IDX_BA+1, 7) = 1.0;
    G(IDX_BA+2, 8) = 1.0;
    G(IDX_BG,   9)  = 1.0;
    G(IDX_BG+1, 10) = 1.0;
    G(IDX_BG+2, 11) = 1.0;
    G(IDX_BBARO, 12) = 1.0;

    // --- Van Loan Discretization (use scratch) ---
    auto& A = scratch_.vl_A;
    auto& B = scratch_.vl_B;
    auto& A2 = scratch_.vl_A2;
    auto& GQGt = scratch_.GQGt;
    
    GQGt.noalias() = G * Qc_ * G.transpose();

    A.setZero();
    A.block<DIM_STATE, DIM_STATE>(0, 0) = -F * dt;
    A.block<DIM_STATE, DIM_STATE>(0, DIM_STATE) = GQGt * dt;
    A.block<DIM_STATE, DIM_STATE>(DIM_STATE, DIM_STATE) = F.transpose() * dt;

    // Taylor expansion: B = I + A + 0.5*A*A
    A2.noalias() = A * A;
    B = A;
    B += 0.5 * A2;
    // Add identity (just diagonal)
    for (int i = 0; i < 2*DIM_STATE; ++i) {
        B(i, i) += 1.0;
    }

    auto& Phi = scratch_.Phi;
    auto& Qd = scratch_.Qd;
    
    Phi = B.block<DIM_STATE, DIM_STATE>(DIM_STATE, DIM_STATE).transpose();
    Qd.noalias() = Phi * B.block<DIM_STATE, DIM_STATE>(0, DIM_STATE);
    
    // Symmetrize
    Qd = 0.5 * (Qd + Qd.transpose());

    // Covariance propagation: P = Phi * P * Phi' + Qd
    auto& Phi_P = scratch_.Phi_P;
    Phi_P.noalias() = Phi * P_;
    P_.noalias() = Phi_P * Phi.transpose();
    P_ += Qd;
}

void EKF16d_OPT::inject_error(const Eigen::Matrix<double, DIM_STATE, 1>& dx) {
    x_.p += dx.segment<3>(IDX_POS);
    x_.v += dx.segment<3>(IDX_VEL);
    x_.ba += dx.segment<3>(IDX_BA);
    x_.bg += dx.segment<3>(IDX_BG);
    x_.bbaro += dx(IDX_BBARO);

    double dtheta_x = dx(IDX_ATT);
    double dtheta_y = dx(IDX_ATT + 1);
    double dtheta_z = dx(IDX_ATT + 2);
    double theta_norm = sqrt(dtheta_x*dtheta_x + dtheta_y*dtheta_y + dtheta_z*dtheta_z);
    
    Eigen::Quaterniond dq;
    if (theta_norm > 1e-8) {
        double half_angle = 0.5 * theta_norm;
        double s = sin(half_angle) / theta_norm;
        dq = Eigen::Quaterniond(cos(half_angle), s*dtheta_x, s*dtheta_y, s*dtheta_z);
    } else {
        dq = Eigen::Quaterniond(1, 0.5*dtheta_x, 0.5*dtheta_y, 0.5*dtheta_z);
    }
    x_.q = dq * x_.q;
    x_.q.normalize();
}

void EKF16d_OPT::update_gnss_position(const Eigen::Vector3d& pos_gnss, const Eigen::Matrix3d& R) {
    auto& z = scratch_.z3;
    auto& H = scratch_.H3;
    auto& S = scratch_.S3;
    auto& K = scratch_.K3;
    auto& PH_t = scratch_.PH3t;
    auto& I_KH = scratch_.I_KH;
    auto& dx = scratch_.dx;
    
    z = pos_gnss - x_.p;
    
    H.setZero();
    H(0, IDX_POS)   = 1.0;
    H(1, IDX_POS+1) = 1.0;
    H(2, IDX_POS+2) = 1.0;

    // S = H*P*H' + R
    PH_t.noalias() = P_ * H.transpose();
    S.noalias() = H * PH_t;
    S += R;
    
    // K = P*H'*S^-1
    K.noalias() = PH_t * S.inverse();
    
    dx.noalias() = K * z;
    
    // Joseph form: P = (I-KH)*P*(I-KH)' + K*R*K'
    I_KH.noalias() = -K * H;
    for (int i = 0; i < DIM_STATE; ++i) I_KH(i,i) += 1.0;
    
    scratch_.Phi_P.noalias() = I_KH * P_;
    P_.noalias() = scratch_.Phi_P * I_KH.transpose();
    PH_t.noalias() = K * R;  // PH_t is 16x3, reuse it
    P_.noalias() += PH_t * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());

    inject_error(dx);
}

void EKF16d_OPT::update_gnss_velocity(const Eigen::Vector3d& vel_gnss, const Eigen::Matrix3d& R) {
    auto& z = scratch_.z3;
    auto& H = scratch_.H3;
    auto& S = scratch_.S3;
    auto& K = scratch_.K3;
    auto& PH_t = scratch_.PH3t;
    auto& I_KH = scratch_.I_KH;
    auto& dx = scratch_.dx;
    
    z = vel_gnss - x_.v;
    
    H.setZero();
    H(0, IDX_VEL)   = 1.0;
    H(1, IDX_VEL+1) = 1.0;
    H(2, IDX_VEL+2) = 1.0;

    PH_t.noalias() = P_ * H.transpose();
    S.noalias() = H * PH_t;
    S += R;
    
    K.noalias() = PH_t * S.inverse();
    dx.noalias() = K * z;
    
    I_KH.noalias() = -K * H;
    for (int i = 0; i < DIM_STATE; ++i) I_KH(i,i) += 1.0;
    
    scratch_.Phi_P.noalias() = I_KH * P_;
    P_.noalias() = scratch_.Phi_P * I_KH.transpose();
    PH_t.noalias() = K * R;  // PH_t is 16x3, reuse it
    P_.noalias() += PH_t * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());

    inject_error(dx);
}

void EKF16d_OPT::update_barometer(double altitude, double R_var) {
    auto& H = scratch_.H1;
    auto& K = scratch_.K1;
    auto& PH_t = scratch_.PH1t;
    auto& I_KH = scratch_.I_KH;
    auto& dx = scratch_.dx;
    
    double z = altitude - (x_.p.z() + x_.bbaro);
    
    H.setZero();
    H(0, IDX_POS + 2) = 1.0;
    H(0, IDX_BBARO) = 1.0;

    // S = H*P*H' + R (scalar)
    PH_t.noalias() = P_ * H.transpose();
    double S = (H * PH_t)(0,0) + R_var;
    
    // K = P*H'/S
    K = PH_t * (1.0 / S);
    
    dx = K * z;
    
    // Joseph form
    I_KH.noalias() = -K * H;
    for (int i = 0; i < DIM_STATE; ++i) I_KH(i,i) += 1.0;
    
    scratch_.Phi_P.noalias() = I_KH * P_;
    P_.noalias() = scratch_.Phi_P * I_KH.transpose();
    P_.noalias() += (R_var * K) * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());

    inject_error(dx);
}

void EKF16d_OPT::update_magnetometer(const Eigen::Vector3d& mag_body, const Eigen::Matrix3d& R) {
    static const Eigen::Vector3d m_n(0.40, 0.0, 0.92);
    
    auto& C_b_n = scratch_.C_bn;
    auto& z = scratch_.z3;
    auto& H = scratch_.H3;
    auto& S = scratch_.S3;
    auto& K = scratch_.K3;
    auto& PH_t = scratch_.PH3t;
    auto& I_KH = scratch_.I_KH;
    auto& dx = scratch_.dx;
    
    C_b_n = x_.q.toRotationMatrix();
    
    // Predicted measurement: C_n_b * m_n
    scratch_.v3_a.noalias() = C_b_n.transpose() * m_n;
    
    // Normalize mag_body into z, then subtract prediction
    z = mag_body;
    z.normalize();
    z -= scratch_.v3_a;
    
    // H = C_n_b * skew(m_n)
    H.setZero();
    // skew(m_n) = [0, -m_n.z, m_n.y; m_n.z, 0, -m_n.x; -m_n.y, m_n.x, 0]
    Eigen::Matrix3d skew_m;
    skew_m <<       0, -m_n.z(),  m_n.y(),
              m_n.z(),        0, -m_n.x(),
             -m_n.y(),  m_n.x(),        0;
    H.block<3,3>(0, IDX_ATT) = C_b_n.transpose() * skew_m;
    
    PH_t.noalias() = P_ * H.transpose();
    S.noalias() = H * PH_t;
    S += R;
    
    K.noalias() = PH_t * S.inverse();
    dx.noalias() = K * z;
    
    I_KH.noalias() = -K * H;
    for (int i = 0; i < DIM_STATE; ++i) I_KH(i,i) += 1.0;
    
    scratch_.Phi_P.noalias() = I_KH * P_;
    P_.noalias() = scratch_.Phi_P * I_KH.transpose();
    PH_t.noalias() = K * R;
    P_.noalias() += PH_t * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());

    inject_error(dx);
}
