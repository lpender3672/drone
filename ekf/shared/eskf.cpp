#include "eskf.h"
#include "tuned_imu_params.h"

using namespace IMUErrorModel;

EsEkf::EsEkf() {
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

void EsEkf::initialize(const Eigen::Vector3d& init_pos, 
                       const Eigen::Vector3d& init_vel, 
                       const Eigen::Quaterniond& init_quat,
                       const Eigen::Vector3d& init_ba,
                       const Eigen::Vector3d& init_bg,
                       double init_bbaro,
                       const Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>& init_P) {
    x_.p = init_pos;
    x_.v = init_vel;
    x_.q = init_quat;
    x_.q.normalize();
    x_.ba = init_ba;
    x_.bg = init_bg;
    x_.bbaro = init_bbaro;
    P_ = init_P;
}

Eigen::Matrix3d EsEkf::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

void EsEkf::compute_radius(double lat, double& RM, double& RN) {
    double sin_lat = sin(lat);
    double sin2_lat = sin_lat * sin_lat;
    double den = 1.0 - e2 * sin2_lat;
    
    RM = (R0 * (1.0 - e2)) / pow(den, 1.5);
    RN = R0 / sqrt(den);
}

void EsEkf::predict(const ImuMeasurement& imu, double dt) {
    // Correct measurements
    Eigen::Vector3d f_b = imu.acc - x_.ba;
    Eigen::Vector3d w_b = imu.gyro - x_.bg;

    Eigen::Matrix3d C_b_n = x_.q.toRotationMatrix();
    double lat = x_.p.x();
    double h = x_.p.z();
    
    double RM, RN;
    compute_radius(lat, RM, RN);

    // Earth rate in nav frame
    Eigen::Vector3d w_ie_n;
    w_ie_n << WE * cos(lat), 0.0, -WE * sin(lat);

    // Transport rate
    Eigen::Vector3d w_en_n;
    w_en_n << x_.v.y() / (RN + h),
             -x_.v.x() / (RM + h),
             -x_.v.x() * x_.v.y() * tan(lat) / (RN + h);

    Eigen::Vector3d w_in_n = w_ie_n + w_en_n;
    
    // Gravity (Somigliana)
    Eigen::Vector3d g_n(0, 0, 
        9.780327 * (1 + 0.0053024 * sin(lat)*sin(lat) - 0.0000058 * sin(2*lat)*sin(2*lat)) 
        - 3.086e-6 * h);

    // Velocity mechanization
    Eigen::Vector3d v_dot = C_b_n * f_b + g_n - (2.0 * w_ie_n + w_en_n).cross(x_.v);
    Eigen::Vector3d v_next = x_.v + v_dot * dt;

    // Position mechanization
    Eigen::Vector3d p_dot;
    p_dot << x_.v.x() / (RM + h),
             x_.v.y() / ((RN + h) * cos(lat)),
             -x_.v.z();
    
    x_.p.x() += p_dot.x() * dt;
    x_.p.y() += p_dot.y() * dt;
    x_.p.z() -= x_.v.z() * dt;
    x_.v = v_next;

    // Attitude mechanization
    Eigen::Vector3d w_nb_b = w_b - C_b_n.transpose() * w_in_n;
    Eigen::Vector3d angle = w_nb_b * dt;
    double angle_norm = angle.norm();
    Eigen::Quaterniond dq;
    if (angle_norm > 1e-8) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle_norm, angle / angle_norm));
    } else {
        dq = Eigen::Quaterniond(1, 0.5*angle.x(), 0.5*angle.y(), 0.5*angle.z());
        dq.normalize();
    }
    x_.q = x_.q * dq;
    x_.q.normalize();

    // Baro bias: no dynamics on nominal state (bias just persists)

    // --- Build F Matrix ---
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> F = Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>::Zero();

    // F_rv (Pos-Vel)
    F(IDX_POS, IDX_VEL)     = 1.0 / (RM + h);
    F(IDX_POS+1, IDX_VEL+1) = 1.0 / ((RN + h) * cos(lat));
    F(IDX_POS+2, IDX_VEL+2) = -1.0;

    // F_vv (Vel-Vel) - Coriolis
    Eigen::Vector3d w_term = 2.0 * w_ie_n + w_en_n;
    F.block<3,3>(IDX_VEL, IDX_VEL) = -skew(w_term);

    // F_v_theta (Vel-Att)
    Eigen::Vector3d f_n = C_b_n * f_b;
    F.block<3,3>(IDX_VEL, IDX_ATT) = -skew(f_n);

    // F_v_ba (Vel-AccelBias)
    F.block<3,3>(IDX_VEL, IDX_BA) = -C_b_n;

    // F_theta_theta (Att-Att)
    F.block<3,3>(IDX_ATT, IDX_ATT) = -skew(w_in_n);

    // F_theta_bg (Att-GyroBias)
    F.block<3,3>(IDX_ATT, IDX_BG) = -C_b_n;

    // Gauss-Markov bias dynamics
    F.block<3,3>(IDX_BA, IDX_BA).diagonal() = -1.0 / tau_a_.array();
    F.block<3,3>(IDX_BG, IDX_BG).diagonal() = -1.0 / tau_g_.array();
    F(IDX_BBARO, IDX_BBARO) = -1.0 / tau_bbaro_;

    // --- Build G Matrix ---
    Eigen::Matrix<double, DIM_ERROR, DIM_NOISE> G = Eigen::Matrix<double, DIM_ERROR, DIM_NOISE>::Zero();
    G.block<3,3>(IDX_VEL, 0) = -C_b_n;                        // vel - accel noise
    G.block<3,3>(IDX_ATT, 3) = -C_b_n;                        // att - gyro noise
    G.block<3,3>(IDX_BA, 6)  = Eigen::Matrix3d::Identity();   // ba - driving noise
    G.block<3,3>(IDX_BG, 9)  = Eigen::Matrix3d::Identity();   // bg - driving noise
    G(IDX_BBARO, 12) = 1.0;                                   // bbaro - driving noise

    // --- Van Loan Discretization ---
    Eigen::Matrix<double, 2*DIM_ERROR, 2*DIM_ERROR> A;
    A.setZero();
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> GQGt = G * Qc_ * G.transpose();

    A.block<DIM_ERROR, DIM_ERROR>(0, 0) = -F * dt;
    A.block<DIM_ERROR, DIM_ERROR>(0, DIM_ERROR) = GQGt * dt;
    A.block<DIM_ERROR, DIM_ERROR>(DIM_ERROR, DIM_ERROR) = F.transpose() * dt;

    // Taylor expansion for matrix exponential
    Eigen::Matrix<double, 2*DIM_ERROR, 2*DIM_ERROR> I_vl = 
        Eigen::Matrix<double, 2*DIM_ERROR, 2*DIM_ERROR>::Identity();
    //Eigen::Matrix<double, 2*DIM_ERROR, 2*DIM_ERROR> B = I_vl + A + 0.5 * A * A + (1.0/6.0) * A * A * A + (1.0/24.0) * A * A * A * A;

    Eigen::Matrix<double, 2*DIM_ERROR, 2*DIM_ERROR> B = A.exp();

    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> Phi = 
        B.block<DIM_ERROR, DIM_ERROR>(DIM_ERROR, DIM_ERROR).transpose();
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> Qd = 
        Phi * B.block<DIM_ERROR, DIM_ERROR>(0, DIM_ERROR);

    Qd = 0.5 * (Qd + Qd.transpose());

    P_ = Phi * P_ * Phi.transpose() + Qd;
}

void EsEkf::inject_error(const Eigen::Matrix<double, DIM_ERROR, 1>& dx) {
    x_.p += dx.segment<3>(IDX_POS);
    x_.v += dx.segment<3>(IDX_VEL);
    x_.ba += dx.segment<3>(IDX_BA);
    x_.bg += dx.segment<3>(IDX_BG);
    x_.bbaro += dx(IDX_BBARO);

    // Attitude correction
    Eigen::Vector3d dtheta = dx.segment<3>(IDX_ATT);
    double theta_norm = dtheta.norm();
    Eigen::Quaterniond dq;
    if(theta_norm > 1e-8) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta_norm, dtheta / theta_norm));
    } else {
        dq = Eigen::Quaterniond(1, 0.5*dtheta.x(), 0.5*dtheta.y(), 0.5*dtheta.z());
    }
    x_.q = dq * x_.q;
    x_.q.normalize();
}

template<int M>
void EsEkf::update_internal(
    const Eigen::Matrix<double, M, 1>& z,
    const Eigen::Matrix<double, M, DIM_ERROR>& H,
    const Eigen::Matrix<double, M, M>& R)
{
    Eigen::Matrix<double, M, M> S =
        H * P_ * H.transpose() + R;

    Eigen::Matrix<double, DIM_ERROR, M> K =
        P_ * H.transpose() * S.inverse();

    Eigen::Matrix<double, DIM_ERROR, 1> dx = K * z;

    // Joseph form
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> I_KH =
        Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>::Identity()
        - K * H;

    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());

    inject_error(dx);
}

void EsEkf::update_gnss_position(const Eigen::Vector3d& pos_gnss, const Eigen::Matrix3d& R) {
    Eigen::Vector3d innovation = pos_gnss - x_.p;
    
    Eigen::Matrix<double, 3, DIM_ERROR> H;
    H.setZero();
    H.block<3,3>(0, IDX_POS) = Eigen::Matrix3d::Identity();

    update_internal<3>(innovation, H, R);
}

void EsEkf::update_gnss_velocity(const Eigen::Vector3d& vel_gnss, const Eigen::Matrix3d& R) {
    Eigen::Vector3d innovation = vel_gnss - x_.v;

    Eigen::Matrix<double, 3, DIM_ERROR> H;
    H.setZero();
    H.block<3,3>(0, IDX_VEL) = Eigen::Matrix3d::Identity();

    update_internal<3>(innovation, H, R);
}

void EsEkf::update_barometer(double altitude, double R_var) {
    double innovation = altitude - (x_.p.z() + x_.bbaro);

    // H = [0 0 1 | 0 0 0 | 0 0 0 | 0 0 0 | 0 0 0 | 1]
    //      pos      vel     att     ba      bg    bbaro
    Eigen::Matrix<double, 1, DIM_ERROR> H;
    H.setZero();
    H(0, IDX_POS + 2) = 1.0;   // dh
    H(0, IDX_BBARO) = 1.0;     // dbbaro

    Eigen::Matrix<double, 1, 1> R_mat;
    R_mat(0,0) = R_var;

    Eigen::VectorXd z(1);
    z(0) = innovation;

    update_internal<1>(z, H, R_mat);
}

void EsEkf::update_magnetometer(const Eigen::Vector3d& mag_body, 
                                 const Eigen::Matrix3d& R) {

    // apparently the magnetic field vector in southend on sea
    static const Eigen::Vector3d m_n(0.40, 0.0, 0.92);
    
    Eigen::Matrix3d C_b_n = x_.q.toRotationMatrix();
    Eigen::Matrix3d C_n_b = C_b_n.transpose();
    
    // Predicted measurement
    Eigen::Vector3d mag_pred = C_n_b * m_n;
    
    // Innovation
    Eigen::Vector3d innovation = mag_body.normalized() - mag_pred;
    
    // H matrix: d(mag_pred)/d(theta) = -C_n_b * skew(m_n)
    Eigen::Matrix<double, 3, DIM_ERROR> H;
    H.setZero();
    H.block<3,3>(0, IDX_ATT) = C_n_b * skew(m_n);
    
    update_internal<3>(innovation, H, R);
}

EkfStatus EsEkf::getStatus(double max_variance_threshold) const {
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
