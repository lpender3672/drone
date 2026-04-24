#include "ekf16.h"

template<typename Scalar>
EKF16<Scalar>::EKF16(const EkfErrorParameters& p)
    : EKF<Scalar, DIM_NOMINAL, DIM_ERROR, DIM_NOISE>(p)
{
    P_.setIdentity();
    Qc_.setZero();

    // --- Gyro parameters (rad/s units) ---
    Vec3 N_gyro(Scalar(p.gyro_x_n), Scalar(p.gyro_y_n), Scalar(p.gyro_z_n));
    Vec3 B_gyro(Scalar(p.gyro_x_b), Scalar(p.gyro_y_b), Scalar(p.gyro_z_b));
    Vec3 Tp_gyro(Scalar(p.gyro_x_tp), Scalar(p.gyro_y_tp), Scalar(p.gyro_z_tp));

    // --- Accelerometer parameters (m/s^2 units) ---
    Vec3 N_acc(Scalar(p.accel_x_n), Scalar(p.accel_y_n), Scalar(p.accel_z_n));
    Vec3 B_acc(Scalar(p.accel_x_b), Scalar(p.accel_y_b), Scalar(p.accel_z_b));
    Vec3 Tp_acc(Scalar(p.accel_x_tp), Scalar(p.accel_y_tp), Scalar(p.accel_z_tp));

    // --- Barometer parameters (m units) ---
    const Scalar N_baro  = Scalar(p.baro_altitude_n);
    const Scalar B_baro  = Scalar(p.baro_altitude_b);
    const Scalar Tp_baro = Scalar(p.baro_altitude_tp);

    baro_noise_var_ = N_baro * N_baro;
    gravity_noise_var_ = Vec3(
        Scalar(p.gravity_sigma_x) * Scalar(p.gravity_sigma_x),
        Scalar(p.gravity_sigma_y) * Scalar(p.gravity_sigma_y),
        Scalar(p.gravity_sigma_z) * Scalar(p.gravity_sigma_z));

    // --- Gauss-Markov time constants (Tp / 1.89 per Farrell eqn 34) ---
    tau_g_     = Tp_gyro / Scalar(1.89);
    tau_a_     = Tp_acc  / Scalar(1.89);
    tau_bbaro_ = Tp_baro / Scalar(1.89);

    // --- Driving noise variances ---
    constexpr Scalar k = Scalar(0.4365);

    Vec3 sigma_ba = B_acc.array().square() / (tau_a_.array() * k * k);
    Vec3 sigma_bg = B_gyro.array().square() / (tau_g_.array() * k * k);
    const Scalar sigma_bbaro = (B_baro * B_baro) / (tau_bbaro_ * k * k);

    Vec3 q_ba = Scalar(2.0) * sigma_ba.array().square() / tau_a_.array();
    Vec3 q_bg = Scalar(2.0) * sigma_bg.array().square() / tau_g_.array();
    const Scalar q_bbaro = Scalar(2.0) * sigma_bbaro * sigma_bbaro / tau_bbaro_;

    // --- Continuous-time process noise matrix ---
    Qc_.diagonal() <<
        N_gyro.array().square(),  // 0-2: gyro white noise
        N_acc.array().square(),   // 3-5: accel white noise
        q_ba,                     // 6-8: accel bias driving noise
        q_bg,                     // 9-11: gyro bias driving noise
        q_bbaro;                  // 12: baro bias driving noise
}

template<typename Scalar>
void EKF16<Scalar>::predict(const ImuMeasurement& imu, double dt_d) {
    reserver_.reset();

    if (debugCallback) debugCallback("Before state prediction");

    const Scalar dt = Scalar(dt_d);

    Vec3 f_b = imu.acc.template cast<Scalar>()  - ba();
    Vec3 w_b = imu.gyro.template cast<Scalar>() - bg();

    Quat qn = quat_from_state(q_vec());
    Mat3 C_b_n = qn.toRotationMatrix();

    Scalar lat = pos()(0);
    Scalar h   = pos()(2);

    Scalar RM, RN;
    compute_radius<Scalar>(lat, RM, RN);

    Vec3 w_ie_n(WE_ * std::cos(lat), Scalar(0), -WE_ * std::sin(lat));
    Vec3 w_en_n(
        vel()(1) / (RN + h),
        -vel()(0) / (RM + h),
        -vel()(0) * vel()(1) * std::tan(lat) / (RN + h)
    );
    Vec3 w_in_n = w_ie_n + w_en_n;

    Vec3 g_n = gravity_ned<Scalar>(lat, h);

    // Velocity
    Vec3 v_dot = C_b_n * f_b + g_n - (Scalar(2.0) * w_ie_n + w_en_n).cross(vel());
    Vec3 v_next = vel() + v_dot * dt;

    // Position (geodetic: lat_rad, lon_rad, alt_m)
    pos()(0) += vel()(0) / (RM + h) * dt;
    pos()(1) += vel()(1) / ((RN + h) * std::cos(lat)) * dt;
    pos()(2) -= vel()(2) * dt;
    vel() = v_next;

    // Attitude
    Vec3 w_nb_b = w_b - C_b_n.transpose() * w_in_n;
    Vec3 angle = w_nb_b * dt;

    Quat dq;
    Scalar a = angle.norm();
    if (a > Scalar(1e-8)) {
        dq = Quat(Eigen::AngleAxis<Scalar>(a, angle / a));
    } else {
        dq = Quat(Scalar(1), Scalar(0.5) * angle.x(), Scalar(0.5) * angle.y(), Scalar(0.5) * angle.z());
    }

    qn = qn * dq;
    qn.normalize();
    q_vec() = state_from_quat(qn);

    if (debugCallback) debugCallback("After state prediction");

    // --- Build F Matrix ---
    auto F = reserver_.template matrix<DIM_ERROR, DIM_ERROR>();
    F.setZero();

    F(ERR_POS, ERR_VEL)     = Scalar(1) / (RM + h);
    F(ERR_POS+1, ERR_VEL+1) = Scalar(1) / ((RN + h) * std::cos(lat));
    F(ERR_POS+2, ERR_VEL+2) = Scalar(-1);

    Vec3 w_term = Scalar(2) * w_ie_n + w_en_n;
    F.template block<3,3>(ERR_VEL, ERR_VEL) = -skew(w_term);

    Vec3 f_n = C_b_n * f_b;
    F.template block<3,3>(ERR_VEL, ERR_ATT) = -skew(f_n);
    F.template block<3,3>(ERR_VEL, ERR_BA) = -C_b_n;
    F.template block<3,3>(ERR_ATT, ERR_ATT) = -skew(w_in_n);
    F.template block<3,3>(ERR_ATT, ERR_BG) = -C_b_n;

    F.template block<3,3>(ERR_BA, ERR_BA).diagonal() = Scalar(-1) / tau_a_.array();
    F.template block<3,3>(ERR_BG, ERR_BG).diagonal() = Scalar(-1) / tau_g_.array();
    F(ERR_BBARO, ERR_BBARO) = Scalar(-1) / tau_bbaro_;

    if (debugCallback) debugCallback("After F matrix build");

    // --- Build G Matrix ---
    auto G = reserver_.template matrix<DIM_ERROR, DIM_NOISE>();
    G.setZero();
    G.template block<3,3>(ERR_VEL, 0) = -C_b_n;
    G.template block<3,3>(ERR_ATT, 3) = -C_b_n;
    G.template block<3,3>(ERR_BA, 6).setIdentity();
    G.template block<3,3>(ERR_BG, 9).setIdentity();
    G(ERR_BBARO, 12) = Scalar(1);

    if (debugCallback) debugCallback("After G matrix build");

    // --- Van Loan Discretization (2nd-order Taylor) ---
    auto GQGt = reserver_.template matrix<DIM_ERROR, DIM_ERROR>();
    GQGt.noalias() = G * Qc_ * G.transpose();

    auto A = reserver_.template matrix<2*DIM_ERROR, 2*DIM_ERROR>();
    A.setZero();
    A.template block<DIM_ERROR, DIM_ERROR>(0, 0) = -F * dt;
    A.template block<DIM_ERROR, DIM_ERROR>(0, DIM_ERROR) = GQGt * dt;
    A.template block<DIM_ERROR, DIM_ERROR>(DIM_ERROR, DIM_ERROR) = F.transpose() * dt;

    auto A2 = reserver_.template matrix<2*DIM_ERROR, 2*DIM_ERROR>();
    A2.noalias() = A * A;

    auto B = reserver_.template matrix<2*DIM_ERROR, 2*DIM_ERROR>();
    B.setIdentity();
    B += A;
    B += Scalar(0.5) * A2;

    auto Phi = reserver_.template matrix<DIM_ERROR, DIM_ERROR>();
    Phi = B.template block<DIM_ERROR, DIM_ERROR>(DIM_ERROR, DIM_ERROR).transpose();

    auto Qd = reserver_.template matrix<DIM_ERROR, DIM_ERROR>();
    Qd.noalias() = Phi * B.template block<DIM_ERROR, DIM_ERROR>(0, DIM_ERROR);
    Qd = Scalar(0.5) * (Qd + Qd.transpose()).eval();

    auto Phi_P = reserver_.template matrix<DIM_ERROR, DIM_ERROR>();
    Phi_P.noalias() = Phi * P_;
    P_.noalias() = Phi_P * Phi.transpose() + Qd;

    if (debugCallback) debugCallback("After covariance prediction");
}

template<typename Scalar>
void EKF16<Scalar>::inject_error(const ErrorVector& dx) {
    // Additive states
    x_.template segment<3>(NOM_POS) += dx.template segment<3>(ERR_POS);
    x_.template segment<3>(NOM_VEL) += dx.template segment<3>(ERR_VEL);
    x_.template segment<3>(NOM_BA)  += dx.template segment<3>(ERR_BA);
    x_.template segment<3>(NOM_BG)  += dx.template segment<3>(ERR_BG);
    x_(NOM_BBARO)                    += dx(ERR_BBARO);

    // Attitude (multiplicative, left-multiply in NED)
    Vec3 dtheta = dx.template segment<3>(ERR_ATT);
    Scalar a = dtheta.norm();

    Quat dq;
    if (a > Scalar(1e-8)) {
        dq = Quat(Eigen::AngleAxis<Scalar>(a, dtheta / a));
    } else {
        dq = Quat(Scalar(1), Scalar(0.5) * dtheta.x(), Scalar(0.5) * dtheta.y(), Scalar(0.5) * dtheta.z());
    }

    Quat qn(x_(NOM_QUAT + 0), x_(NOM_QUAT + 1), x_(NOM_QUAT + 2), x_(NOM_QUAT + 3));
    qn = dq * qn;
    qn.normalize();

    x_.template segment<4>(NOM_QUAT) << qn.w(), qn.x(), qn.y(), qn.z();
}

template<typename Scalar>
void EKF16<Scalar>::update_gnss_position(const Vec3& pos_gnss, const Mat3& R) {
    if (debugCallback) debugCallback("Enter GNSS pos update");
    reserver_.reset();

    Vec3 innovation = pos_gnss - pos();

    auto H = reserver_.template matrix<3, DIM_ERROR>();
    H.setZero();
    H.template block<3,3>(0, ERR_POS).setIdentity();

    if (debugCallback) debugCallback("Before GNSS pos update");
    this->template update_internal<3>(innovation, H, R);
}

template<typename Scalar>
void EKF16<Scalar>::update_gnss_velocity(const Vec3& vel_gnss, const Mat3& R) {
    if (debugCallback) debugCallback("Enter GNSS vel update");
    reserver_.reset();

    Vec3 innovation = vel_gnss - vel();

    auto H = reserver_.template matrix<3, DIM_ERROR>();
    H.setZero();
    H.template block<3,3>(0, ERR_VEL).setIdentity();

    if (debugCallback) debugCallback("Before GNSS vel update");
    this->template update_internal<3>(innovation, H, R);
}

template<typename Scalar>
void EKF16<Scalar>::update_barometer(Scalar altitude, Scalar R_var) {
    reserver_.reset();
    Scalar innovation = altitude - (pos().z() + bbaro());

    // H = [0 0 1 | 0 0 0 | 0 0 0 | 0 0 0 | 0 0 0 | 1]
    //      pos      vel     att     ba      bg    bbaro
    auto H = reserver_.template matrix<1, DIM_ERROR>();
    H.setZero();
    H(0, ERR_POS + 2) = Scalar(1);
    H(0, ERR_BBARO)   = Scalar(1);

    Eigen::Matrix<Scalar, 1, 1> R_mat;
    R_mat(0, 0) = R_var;

    Eigen::Matrix<Scalar, 1, 1> z;
    z(0) = innovation;

    if (debugCallback) debugCallback("Before baro update");
    this->template update_internal<1>(z, H, R_mat);
}

template<typename Scalar>
void EKF16<Scalar>::update_magnetometer(const Vec3& mag_body, const Mat3& R) {
    reserver_.reset();

    if (debugCallback) debugCallback("Before mag update");

    Mat3 C_b_n = quat_from_state(q_vec()).toRotationMatrix();
    Mat3 C_n_b = C_b_n.transpose();

    Vec3 mag_pred = C_n_b * mag_reference_;
    Vec3 innovation = mag_body.normalized() - mag_pred;

    auto H = reserver_.template matrix<3, DIM_ERROR>();
    H.setZero();
    H.template block<3,3>(0, ERR_ATT) = C_n_b * skew(mag_reference_);

    if (debugCallback) debugCallback("After mag H matrix build");
    this->template update_internal<3>(innovation, H, R);
}

template<typename Scalar>
void EKF16<Scalar>::update_gravity(const Vec3& f_body) {
    reserver_.reset();

    // Gravity aiding: at hover, f_body = -C_n_b * g * g_n, so -f_body.normalized() = C_n_b * g_n.
    // Innovation = 0 at the true state.
    // H derivation (same as magnetometer, substituting g_n): H_att = C_n_b * skew(g_n).
    static const Vec3 g_n(Scalar(0), Scalar(0), Scalar(1));

    Mat3 C_b_n = quat_from_state(q_vec()).toRotationMatrix();
    Mat3 C_n_b = C_b_n.transpose();

    Vec3 g_pred = C_n_b * g_n;
    Vec3 g_meas = -f_body.normalized();
    Vec3 innovation = g_meas - g_pred;

    auto H = reserver_.template matrix<3, DIM_ERROR>();
    H.setZero();
    H.template block<3,3>(0, ERR_ATT) = C_n_b * skew(g_n);

    Mat3 R = gravity_noise_var_.asDiagonal();

    this->template update_internal<3>(innovation, H, R);
}

// Explicit instantiations — only these two types are supported.
template class EKF16<double>;
template class EKF16<float>;
