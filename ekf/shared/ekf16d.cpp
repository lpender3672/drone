#include "ekf16d.h"
#include "tuned_ekf_params.h"


EKF16d::EKF16d(const EkfErrorParameters& p) : EKF<DIM_STATE, DIM_NOISE>(p)
{
    P_.setIdentity();
    Qc_.setZero();

    // --- Gyro parameters (rad/s units) ---
    Eigen::Vector3d N_gyro(
        p.gyro_x_n,
        p.gyro_y_n,
        p.gyro_z_n
    );

    Eigen::Vector3d B_gyro(
        p.gyro_x_b,
        p.gyro_y_b,
        p.gyro_z_b
    );

    Eigen::Vector3d Tp_gyro(
        p.gyro_x_tp,
        p.gyro_y_tp,
        p.gyro_z_tp
    );

    // --- Accelerometer parameters (m/s^2 units) ---
    Eigen::Vector3d N_acc(
        p.accel_x_n,
        p.accel_y_n,
        p.accel_z_n
    );

    Eigen::Vector3d B_acc(
        p.accel_x_b,
        p.accel_y_b,
        p.accel_z_b
    );

    Eigen::Vector3d Tp_acc(
        p.accel_x_tp,
        p.accel_y_tp,
        p.accel_z_tp
    );

    // --- Barometer parameters (m units) ---
    const double N_baro  = p.baro_altitude_n;
    const double B_baro  = p.baro_altitude_b;
    const double Tp_baro = p.baro_altitude_tp;

    // --- Gauss–Markov time constants ---
    tau_g_      = Tp_gyro / 1.89;
    tau_a_      = Tp_acc  / 1.89;
    tau_bbaro_  = Tp_baro / 1.89;

    // --- Driving noise variances ---
    constexpr double k = 0.4365;

    Eigen::Vector3d sigma_ba =
        B_acc.array().square() / (tau_a_.array() * k * k);

    Eigen::Vector3d sigma_bg =
        B_gyro.array().square() / (tau_g_.array() * k * k);

    const double sigma_bbaro =
        (B_baro * B_baro) / (tau_bbaro_ * k * k);

    Eigen::Vector3d q_ba =
        2.0 * sigma_ba.array().square() / tau_a_.array();

    Eigen::Vector3d q_bg =
        2.0 * sigma_bg.array().square() / tau_g_.array();

    const double q_bbaro =
        2.0 * sigma_bbaro * sigma_bbaro / tau_bbaro_;

    // --- Continuous-time process noise matrix ---
    Qc_.diagonal() <<
        N_gyro.array().square(),  // 0–2: gyro white noise
        N_acc.array().square(),   // 3–5: accel white noise
        q_ba,                     // 6–8: accel bias driving noise
        q_bg,                     // 9–11: gyro bias driving noise
        q_bbaro;                  // 12: baro bias driving noise
}


void EKF16d::compute_radius(double lat, double& RM, double& RN) {
    double sin_lat = sin(lat);
    double sin2_lat = sin_lat * sin_lat;
    double den = 1.0 - e2 * sin2_lat;
    
    RM = (R0 * (1.0 - e2)) / pow(den, 1.5);
    RN = R0 / sqrt(den);
}

void EKF16d::predict(const ImuMeasurement& imu, double dt) {
    Eigen::Vector3d f_b = imu.acc - ba();
    Eigen::Vector3d w_b = imu.gyro - bg();

    Eigen::Matrix3d C_b_n = q_.toRotationMatrix();
    double lat = pos()(0);
    double h = pos()(2);
    
    double RM, RN;
    compute_radius(lat, RM, RN);

    Eigen::Vector3d w_ie_n(WE * cos(lat), 0.0, -WE * sin(lat));

    Eigen::Vector3d w_en_n(
        vel()(1) / (RN + h),
        -vel()(0) / (RM + h),
        -vel()(0) * vel()(1) * tan(lat) / (RN + h)
    );

    Eigen::Vector3d w_in_n = w_ie_n + w_en_n;
    
    Eigen::Vector3d g_n(0, 0, 
        9.780327 * (1 + 0.0053024 * sin(lat)*sin(lat) - 0.0000058 * sin(2*lat)*sin(2*lat)) 
        - 3.086e-6 * h);

    // Velocity mechanization
    Eigen::Vector3d v_dot = C_b_n * f_b + g_n - (2.0 * w_ie_n + w_en_n).cross(vel());
    Eigen::Vector3d v_next = vel() + v_dot * dt;

    // Position mechanization
    pos()(0) += vel()(0) / (RM + h) * dt;
    pos()(1) += vel()(1) / ((RN + h) * cos(lat)) * dt;
    pos()(2) -= vel()(2) * dt;
    vel() = v_next;

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
    q_ = q_ * dq;
    q_.normalize();

    // Baro bias: no dynamics on nominal state (bias just persists)

    // --- Build F Matrix ---
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> F = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Zero();

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
    Eigen::Matrix<double, DIM_STATE, DIM_NOISE> G = Eigen::Matrix<double, DIM_STATE, DIM_NOISE>::Zero();
    G.block<3,3>(IDX_VEL, 0) = -C_b_n;                        // vel - accel noise
    G.block<3,3>(IDX_ATT, 3) = -C_b_n;                        // att - gyro noise
    G.block<3,3>(IDX_BA, 6)  = Eigen::Matrix3d::Identity();   // ba - driving noise
    G.block<3,3>(IDX_BG, 9)  = Eigen::Matrix3d::Identity();   // bg - driving noise
    G(IDX_BBARO, 12) = 1.0;                                   // bbaro - driving noise

    // --- Van Loan Discretization ---
    Eigen::Matrix<double, 2*DIM_STATE, 2*DIM_STATE> A;
    A.setZero();
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> GQGt = G * Qc_ * G.transpose();

    A.block<DIM_STATE, DIM_STATE>(0, 0) = -F * dt;
    A.block<DIM_STATE, DIM_STATE>(0, DIM_STATE) = GQGt * dt;
    A.block<DIM_STATE, DIM_STATE>(DIM_STATE, DIM_STATE) = F.transpose() * dt;

    // Taylor expansion for matrix exponential
    Eigen::Matrix<double, 2*DIM_STATE, 2*DIM_STATE> I_vl = 
        Eigen::Matrix<double, 2*DIM_STATE, 2*DIM_STATE>::Identity();
    Eigen::Matrix<double, 2*DIM_STATE, 2*DIM_STATE> B = I_vl + A + 0.5 * A * A; //+ (1.0/6.0) * A * A * A + (1.0/24.0) * A * A * A * A;

    //Eigen::Matrix<double, 2*DIM_STATE, 2*DIM_STATE> B = A.exp();

    Eigen::Matrix<double, DIM_STATE, DIM_STATE> Phi = 
        B.block<DIM_STATE, DIM_STATE>(DIM_STATE, DIM_STATE).transpose();
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> Qd = 
        Phi * B.block<DIM_STATE, DIM_STATE>(0, DIM_STATE);

    Qd = 0.5 * (Qd + Qd.transpose());

    P_ = Phi * P_ * Phi.transpose() + Qd;
}

void EKF16d::inject_error(const StateVector& dx) {
    pos() += dx.segment<3>(IDX_POS);
    vel() += dx.segment<3>(IDX_VEL);
    ba()  += dx.segment<3>(IDX_BA);
    bg()  += dx.segment<3>(IDX_BG);
    bbaro() += dx(IDX_BBARO);

    // Attitude correction
    Eigen::Vector3d dtheta = dx.segment<3>(IDX_ATT);
    double theta_norm = dtheta.norm();
    Eigen::Quaterniond dq;
    if (theta_norm > 1e-8) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta_norm, dtheta / theta_norm));
    } else {
        dq = Eigen::Quaterniond(1, 0.5*dtheta.x(), 0.5*dtheta.y(), 0.5*dtheta.z());
    }
    q_ = dq * q_;
    q_.normalize();
}

void EKF16d::update_gnss_position(const Eigen::Vector3d& pos_gnss, const Eigen::Matrix3d& R) {
    Eigen::Vector3d innovation = pos_gnss - pos();
    
    Eigen::Matrix<double, 3, DIM_STATE> H;
    H.setZero();
    H.block<3,3>(0, IDX_POS) = Eigen::Matrix3d::Identity();

    update_internal<3>(innovation, H, R);
}

void EKF16d::update_gnss_velocity(const Eigen::Vector3d& vel_gnss, const Eigen::Matrix3d& R) {
    Eigen::Vector3d innovation = vel_gnss - vel();

    Eigen::Matrix<double, 3, DIM_STATE> H;
    H.setZero();
    H.block<3,3>(0, IDX_VEL) = Eigen::Matrix3d::Identity();

    update_internal<3>(innovation, H, R);
}

void EKF16d::update_barometer(double altitude, double R_var) {
    double innovation = altitude - (pos().z() + bbaro());

    // H = [0 0 1 | 0 0 0 | 0 0 0 | 0 0 0 | 0 0 0 | 1]
    //      pos      vel     att     ba      bg    bbaro
    Eigen::Matrix<double, 1, DIM_STATE> H;
    H.setZero();
    H(0, IDX_POS + 2) = 1.0;   // dh
    H(0, IDX_BBARO) = 1.0;     // dbbaro

    Eigen::Matrix<double, 1, 1> R_mat;
    R_mat(0,0) = R_var;

    Eigen::Matrix<double, 1, 1> z;
    z(0) = innovation;

    update_internal<1>(z, H, R_mat);
}

void EKF16d::update_magnetometer(const Eigen::Vector3d& mag_body, 
                                 const Eigen::Matrix3d& R) {

    // apparently the magnetic field vector in southend on sea
    static const Eigen::Vector3d m_n(0.40, 0.0, 0.92);
    
    Eigen::Matrix3d C_b_n = q_.toRotationMatrix();
    Eigen::Matrix3d C_n_b = C_b_n.transpose();
    
    // Predicted measurement
    Eigen::Vector3d mag_pred = C_n_b * m_n;
    
    // Innovation
    Eigen::Vector3d innovation = mag_body.normalized() - mag_pred;
    
    // H matrix: d(mag_pred)/d(theta) = -C_n_b * skew(m_n)
    Eigen::Matrix<double, 3, DIM_STATE> H;
    H.setZero();
    H.block<3,3>(0, IDX_ATT) = C_n_b * skew(m_n);
    
    update_internal<3>(innovation, H, R);
}

