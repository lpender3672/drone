#include "ekf16d.h"
#include "tuned_ekf_params.h"


EKF16d::EKF16d(const EkfErrorParameters& p) : EKF<double, DIM_NOMINAL, DIM_ERROR, DIM_NOISE>(p)
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

    baro_noise_var_ = N_baro * N_baro;
    gravity_noise_var_ = Eigen::Vector3d(
        p.gravity_sigma_x * p.gravity_sigma_x,
        p.gravity_sigma_y * p.gravity_sigma_y,
        p.gravity_sigma_z * p.gravity_sigma_z);

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


void EKF16d::predict(const ImuMeasurement& imu, double dt) {
    reserver_.reset();

    if (debugCallback) debugCallback("Before state prediction");

    Eigen::Vector3d f_b = imu.acc  - ba();
    Eigen::Vector3d w_b = imu.gyro - bg();

    Eigen::Quaterniond qn = quat_from_state(q_vec());
    Eigen::Matrix3d C_b_n = qn.toRotationMatrix();

    double lat = pos()(0);
    double h   = pos()(2);

    double RM, RN;
    compute_radius(lat, RM, RN);

    Eigen::Vector3d w_ie_n(WE * cos(lat), 0.0, -WE * sin(lat));
    Eigen::Vector3d w_en_n(
        vel()(1) / (RN + h),
        -vel()(0) / (RM + h),
        -vel()(0) * vel()(1) * tan(lat) / (RN + h)
    );
    Eigen::Vector3d w_in_n = w_ie_n + w_en_n;

    Eigen::Vector3d g_n = gravity_ned(lat, h);

    // Velocity
    Eigen::Vector3d v_dot =
        C_b_n * f_b + g_n - (2.0 * w_ie_n + w_en_n).cross(vel());
    Eigen::Vector3d v_next = vel() + v_dot * dt;

    // Position
    pos()(0) += vel()(0) / (RM + h) * dt;
    pos()(1) += vel()(1) / ((RN + h) * cos(lat)) * dt;
    pos()(2) -= vel()(2) * dt;
    vel() = v_next;

    // Attitude
    Eigen::Vector3d w_nb_b = w_b - C_b_n.transpose() * w_in_n;
    Eigen::Vector3d angle = w_nb_b * dt;

    Eigen::Quaterniond dq;
    double a = angle.norm();
    if (a > 1e-8) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(a, angle / a));
    } else {
        dq = Eigen::Quaterniond(1, 0.5*angle.x(), 0.5*angle.y(), 0.5*angle.z());
    }

    qn = qn * dq;
    qn.normalize();
    q_vec() = state_from_quat(qn);

    if (debugCallback) debugCallback("After state prediction");

    // --- Build F Matrix ---
    auto F = reserver_.matrix<DIM_ERROR, DIM_ERROR>();
    F.setZero();

    F(ERR_POS, ERR_VEL)     = 1.0 / (RM + h);
    F(ERR_POS+1, ERR_VEL+1) = 1.0 / ((RN + h) * cos(lat));
    F(ERR_POS+2, ERR_VEL+2) = -1.0;

    Eigen::Vector3d w_term = 2.0 * w_ie_n + w_en_n;
    F.block<3,3>(ERR_VEL, ERR_VEL) = -skew(w_term);

    Eigen::Vector3d f_n = C_b_n * f_b;
    F.block<3,3>(ERR_VEL, ERR_ATT) = -skew(f_n);
    F.block<3,3>(ERR_VEL, ERR_BA) = -C_b_n;
    F.block<3,3>(ERR_ATT, ERR_ATT) = -skew(w_in_n);
    F.block<3,3>(ERR_ATT, ERR_BG) = -C_b_n;

    F.block<3,3>(ERR_BA, ERR_BA).diagonal() = -1.0 / tau_a_.array();
    F.block<3,3>(ERR_BG, ERR_BG).diagonal() = -1.0 / tau_g_.array();
    F(ERR_BBARO, ERR_BBARO) = -1.0 / tau_bbaro_;

    if (debugCallback) debugCallback("After F matrix build");

    // --- Build G Matrix ---
    auto G = reserver_.matrix<DIM_ERROR, DIM_NOISE>();
    G.setZero();
    G.block<3,3>(ERR_VEL, 0) = -C_b_n;
    G.block<3,3>(ERR_ATT, 3) = -C_b_n;
    G.block<3,3>(ERR_BA, 6)  = Eigen::Matrix3d::Identity();
    G.block<3,3>(ERR_BG, 9)  = Eigen::Matrix3d::Identity();
    G(ERR_BBARO, 12) = 1.0;

    if (debugCallback) debugCallback("After G matrix build");

    // --- Van Loan Discretization ---
    auto GQGt = reserver_.matrix<DIM_ERROR, DIM_ERROR>();
    GQGt.noalias() = G * Qc_ * G.transpose();

    auto A = reserver_.matrix<2*DIM_ERROR, 2*DIM_ERROR>();
    A.setZero();
    A.template block<DIM_ERROR, DIM_ERROR>(0, 0) = -F * dt;
    A.template block<DIM_ERROR, DIM_ERROR>(0, DIM_ERROR) = GQGt * dt;
    A.template block<DIM_ERROR, DIM_ERROR>(DIM_ERROR, DIM_ERROR) = F.transpose() * dt;

    // Second-order Taylor: B = I + A + 0.5*A²
    auto A2 = reserver_.matrix<2*DIM_ERROR, 2*DIM_ERROR>();
    A2.noalias() = A * A;

    auto B = reserver_.matrix<2*DIM_ERROR, 2*DIM_ERROR>();
    B.setIdentity();
    B += A;
    B += 0.5 * A2;

    // Extract Phi and Qd
    auto Phi = reserver_.matrix<DIM_ERROR, DIM_ERROR>();
    Phi = B.template block<DIM_ERROR, DIM_ERROR>(DIM_ERROR, DIM_ERROR).transpose();

    auto Qd = reserver_.matrix<DIM_ERROR, DIM_ERROR>();
    Qd.noalias() = Phi * B.template block<DIM_ERROR, DIM_ERROR>(0, DIM_ERROR);
    Qd = 0.5 * (Qd + Qd.transpose()).eval();

    // Propagate covariance
    auto Phi_P = reserver_.matrix<DIM_ERROR, DIM_ERROR>();
    Phi_P.noalias() = Phi * P_;
    P_.noalias() = Phi_P * Phi.transpose() + Qd;

    if (debugCallback) debugCallback("After covariance prediction");
}

void EKF16d::inject_error(const ErrorVector& dx) {

    // Additive states
    x_.segment<3>(NOM_POS) += dx.segment<3>(ERR_POS);
    x_.segment<3>(NOM_VEL) += dx.segment<3>(ERR_VEL);
    x_.segment<3>(NOM_BA)  += dx.segment<3>(ERR_BA);
    x_.segment<3>(NOM_BG)  += dx.segment<3>(ERR_BG);
    x_(NOM_BBARO)          += dx(ERR_BBARO);

    // Attitude (multiplicative)
    Eigen::Vector3d dtheta = dx.segment<3>(ERR_ATT);
    double a = dtheta.norm();

    Eigen::Quaterniond dq;
    if (a > 1e-8) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(a, dtheta / a));
    } else {
        dq = Eigen::Quaterniond(
            1,
            0.5*dtheta.x(),
            0.5*dtheta.y(),
            0.5*dtheta.z()
        );
    }

    Eigen::Quaterniond qn(
        x_(NOM_QUAT + 0),
        x_(NOM_QUAT + 1),
        x_(NOM_QUAT + 2),
        x_(NOM_QUAT + 3)
    );

    qn = dq * qn;
    qn.normalize();

    x_.segment<4>(NOM_QUAT) <<
        qn.w(), qn.x(), qn.y(), qn.z();
}


void EKF16d::update_gnss_position(const Eigen::Vector3d& pos_gnss, const Eigen::Matrix3d& R) {

    if (debugCallback) debugCallback("Enter GNSS pos update");

    reserver_.reset();
    Eigen::Vector3d innovation = pos_gnss - pos();
    
    auto H = reserver_.matrix<3, DIM_ERROR>();
    H.setZero();
    H.block<3,3>(0, ERR_POS) = Eigen::Matrix3d::Identity();

    if (debugCallback) debugCallback("Before GNSS pos update");

    update_internal<3>(innovation, H, R);
}

void EKF16d::update_gnss_velocity(const Eigen::Vector3d& vel_gnss, const Eigen::Matrix3d& R) {

    if (debugCallback) debugCallback("Enter GNSS vel update");

    reserver_.reset();
    Eigen::Vector3d innovation = vel_gnss - vel();

    auto H = reserver_.matrix<3, DIM_ERROR>();
    H.setZero();
    H.block<3,3>(0, ERR_VEL) = Eigen::Matrix3d::Identity();

    if (debugCallback) debugCallback("Before GNSS vel update");

    update_internal<3>(innovation, H, R);
}

void EKF16d::update_barometer(double altitude, double R_var) {
    reserver_.reset();
    double innovation = altitude - (pos().z() + bbaro());

    // H = [0 0 1 | 0 0 0 | 0 0 0 | 0 0 0 | 0 0 0 | 1]
    //      pos      vel     att     ba      bg    bbaro
    auto H = reserver_.matrix<1, DIM_ERROR>();
    H.setZero();
    H(0, ERR_POS + 2) = 1.0;   // dh
    H(0, ERR_BBARO) = 1.0;     // dbbaro

    Eigen::Matrix<double, 1, 1> R_mat;
    R_mat(0,0) = R_var;

    Eigen::Matrix<double, 1, 1> z;
    z(0) = innovation;

    if (debugCallback) debugCallback("Before baro update");

    update_internal<1>(z, H, R_mat);
}

void EKF16d::update_magnetometer(const Eigen::Vector3d& mag_body, 
                                 const Eigen::Matrix3d& R) {
    reserver_.reset();

    // apparently the magnetic field vector in southend on sea
    static const Eigen::Vector3d m_n(0.40, 0.0, 0.92);

    if (debugCallback) debugCallback("Before mag update");
    
    Eigen::Matrix3d C_b_n = quat_from_state(q_vec()).toRotationMatrix();
    Eigen::Matrix3d C_n_b = C_b_n.transpose();
    
    // Predicted measurement
    Eigen::Vector3d mag_pred = C_n_b * m_n;
    
    // Innovation
    Eigen::Vector3d innovation = mag_body.normalized() - mag_pred;
    
    // H matrix: d(mag_pred)/d(theta) = -C_n_b * skew(m_n)
    Eigen::Matrix<double, 3, DIM_ERROR> H;
    H.setZero();
    H.block<3,3>(0, ERR_ATT) = C_n_b * skew(m_n);

    if (debugCallback) debugCallback("After mag H matrix build");
    
    update_internal<3>(innovation, H, R);
}

// ============================================================================
// IObserver interface implementation
// ============================================================================

void EKF16d::feed_imu(const sensors::ImuMeasurement& imu) {
    // Compute dt from timestamp
    double dt = last_imu_dt_;
    if (last_imu_timestamp_us_ > 0 && imu.timestamp_us > last_imu_timestamp_us_) {
        dt = (imu.timestamp_us - last_imu_timestamp_us_) * 1e-6;
    }
    last_imu_timestamp_us_ = imu.timestamp_us;
    last_imu_dt_ = dt;
    
    // Cache angular velocity for output() (corrected for bias)
    last_omega_ = imu.gyro - bg();

    // Run prediction step
    predict(imu, dt);

    // Gravity aiding at 50 Hz: use accelerometer to directly constrain roll/pitch.
    // Only when the vehicle is approximately in 1-g flight (not hard maneuvering).
    constexpr uint64_t GRAVITY_PERIOD_US = 10000;  // 100 Hz
    const double f_norm = imu.acc.norm();
    if (last_imu_timestamp_us_ > 0 &&
        (last_imu_timestamp_us_ - last_gravity_update_us_) >= GRAVITY_PERIOD_US &&
        f_norm > 7.0 && f_norm < 14.0)
    {
        last_gravity_update_us_ = last_imu_timestamp_us_;
        update_gravity(imu.acc);
    }
}

void EKF16d::update_gravity(const Eigen::Vector3d& f_body) {
    reserver_.reset();

    // At hover: f_body = -C_n_b * g * g_n, so -f_body.normalized() = C_n_b * g_n
    // Measurement z = -f_body.normalized()  (gravity direction in body frame)
    // Prediction  h = C_n_b * g_n           (same structure as magnetometer)
    // Innovation = 0 at the true state (sign-consistent by construction).
    // H derivation (same as magnetometer, replacing m_n with g_n):
    //   h_true = C_n_b_true * g_n = C_n_b * (I - skew(dth)) * g_n
    //          = C_n_b * g_n + C_n_b * skew(g_n) * dth
    //   H_att = C_n_b * skew(g_n)
    static const Eigen::Vector3d g_n(0.0, 0.0, 1.0);  // unit gravity, NED (down)

    Eigen::Matrix3d C_b_n = quat_from_state(q_vec()).toRotationMatrix();
    Eigen::Matrix3d C_n_b = C_b_n.transpose();

    Eigen::Vector3d g_pred = C_n_b * g_n;
    Eigen::Vector3d g_meas = -f_body.normalized();
    Eigen::Vector3d innovation = g_meas - g_pred;

    Eigen::Matrix<double, 3, DIM_ERROR> H;
    H.setZero();
    H.block<3,3>(0, ERR_ATT) = C_n_b * skew(g_n);

    Eigen::Matrix3d R = gravity_noise_var_.asDiagonal();

    update_internal<3>(innovation, H, R);
}

void EKF16d::feed_mag(const sensors::MagMeasurement& mag) {
    if (!mag.valid) return;

    const double field_norm = mag.field.norm();
    if (field_norm < 10.0) return;  // reject implausible readings (< 10 µT)

    // Normalised-measurement noise: assume ~1 µT body-frame noise.
    // var_normalised = (noise_ut / field_norm)^2 per axis.
    constexpr double noise_ut = 1.0;
    const double n = noise_ut / field_norm;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * (n * n);

    update_magnetometer(mag.field.normalized(), R);
}

void EKF16d::feed_baro(const sensors::BaroMeasurement& baro) {
    if (!baro.valid) return;
    update_barometer(baro.altitude_m, baro_noise_var_);
}

void EKF16d::feed_gnss(const sensors::GnssMeasurement& gnss) {
    if (!gnss.valid || gnss.fix_type < 3) return;

    // Pass geodetic directly: EKF pos state is (lat_rad, lon_rad, alt_m).
    const double lat_rad = gnss.latitude_deg  * (M_PI / 180.0);
    const double lon_rad = gnss.longitude_deg * (M_PI / 180.0);
    Eigen::Vector3d pos_geo(lat_rad, lon_rad, gnss.altitude_m);

    // Horizontal accuracy: prefer receiver-reported 1-sigma if available, else derive from HDOP.
    // Scale from m to rad using meridian (RM) and normal (RN) radii of curvature.
    double RM, RN;
    compute_radius(lat_rad, RM, RN);
    const double cos_lat = std::cos(lat_rad);

    double sigma_h_m = (gnss.h_accuracy_m > 0.0f)
        ? gnss.h_accuracy_m
        : gnss.hdop * std::sqrt(2.5);

    double sigma_v_m = (gnss.v_accuracy_m > 0.0f)
        ? gnss.v_accuracy_m
        : gnss.vdop * 2.0;

    Eigen::Matrix3d R_pos = Eigen::Matrix3d::Zero();
    R_pos(0, 0) = (sigma_h_m * sigma_h_m) / (RM * RM);
    R_pos(1, 1) = (sigma_h_m * sigma_h_m) / (RN * RN * cos_lat * cos_lat);
    R_pos(2, 2) =  sigma_v_m * sigma_v_m;

    update_gnss_position(pos_geo, R_pos);

    // Velocity update — 0.1 m/s std → var = 0.01
    update_gnss_velocity(gnss.velocity_ned, Eigen::Matrix3d::Identity() * 0.01);
}

shared::StateWithBiases EKF16d::output() const {
    shared::StateWithBiases state;
    
    // Position and velocity directly from state vector
    state.position = pos();
    state.velocity = vel();
    
    // Quaternion: EKF stores [w,x,y,z], Eigen Quaterniond uses [x,y,z,w] internally
    // but constructor is (w,x,y,z)
    Eigen::Vector4d qv = q_vec();
    state.attitude = Eigen::Quaterniond(qv(0), qv(1), qv(2), qv(3));
    
    // Angular velocity from cached IMU (bias-corrected)
    state.angular_velocity = last_omega_;
    
    // Biases
    state.accel_bias = ba();
    state.gyro_bias = bg();
    state.baro_bias = bbaro();
    
    return state;
}

void EKF16d::reset(const shared::StateWithBiases& initial) {
    // Build nominal state vector
    NominalVector x0;
    x0.setZero();
    
    x0.segment<3>(NOM_POS) = initial.position;
    x0.segment<3>(NOM_VEL) = initial.velocity;
    
    // Quaternion to state format [w,x,y,z]
    x0(NOM_QUAT + 0) = initial.attitude.w();
    x0(NOM_QUAT + 1) = initial.attitude.x();
    x0(NOM_QUAT + 2) = initial.attitude.y();
    x0(NOM_QUAT + 3) = initial.attitude.z();
    
    x0.segment<3>(NOM_BA) = initial.accel_bias;
    x0.segment<3>(NOM_BG) = initial.gyro_bias;
    x0(NOM_BBARO) = initial.baro_bias;
    
    // Reset covariance to initial values
    CovMatrix P0;
    P0.setIdentity();
    P0.block<3,3>(ERR_POS, ERR_POS) *= 10.0;    // Position uncertainty
    P0.block<3,3>(ERR_VEL, ERR_VEL) *= 1.0;     // Velocity uncertainty
    P0.block<3,3>(ERR_ATT, ERR_ATT) *= 0.1;     // Attitude uncertainty
    P0.block<3,3>(ERR_BA, ERR_BA) *= 0.01;      // Accel bias uncertainty
    P0.block<3,3>(ERR_BG, ERR_BG) *= 0.001;     // Gyro bias uncertainty
    P0(ERR_BBARO, ERR_BBARO) = 1.0;             // Baro bias uncertainty
    
    initialize(x0, P0);
    
    // Reset cached values
    last_omega_ = initial.angular_velocity;
    last_imu_timestamp_us_ = 0;
}
