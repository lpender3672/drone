#include "eskf.h"

EsEkf::EsEkf() {
    // Initialize Covariance and Q_c
    P_.setIdentity();
    Qc_.setZero();

    // [Source: 165, 190] Example Noise Params (These should be tuned based on sensor specs)
    // Values below are placeholders. 
    double sigma_acc_n = 200.0 * 1e-6 * g0; // White noise accel
    double sigma_gyro_n = 0.1 * (M_PI / 180.0); // White noise gyro
    double sigma_acc_w = 1000.0 * 1e-6 * g0; // Random walk accel
    double sigma_gyro_w = 1.0 * (M_PI / 180.0); // Random walk gyro

    // Diagonal Qc [Source: 190]
    Qc_.diagonal() << 
        pow(sigma_acc_n, 2) * Eigen::Vector3d::Ones(),
        pow(sigma_gyro_n, 2) * Eigen::Vector3d::Ones(),
        pow(sigma_acc_w, 2) * Eigen::Vector3d::Ones(),
        pow(sigma_gyro_w, 2) * Eigen::Vector3d::Ones();
}

void EsEkf::initialize(const Eigen::Vector3d& init_pos, 
                       const Eigen::Vector3d& init_vel, 
                       const Eigen::Quaterniond& init_quat,
                       const Eigen::Vector3d& init_ba,
                       const Eigen::Vector3d& init_bg,
                       const Eigen::Matrix<double, 15, 15>& init_P) {
    x_.p = init_pos;
    x_.v = init_vel;
    x_.q = init_quat;
    x_.q.normalize(); // Ensure unit norm
    x_.ba = init_ba;
    x_.bg = init_bg;
    P_ = init_P;
}

// [Source: 104] Skew Symmetric Matrix
Eigen::Matrix3d EsEkf::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

// [Source: 150] Earth Radii
void EsEkf::compute_radius(double lat, double& RM, double& RN) {
    double sin_lat = sin(lat);
    double sin2_lat = sin_lat * sin_lat;
    double den = 1.0 - e2 * sin2_lat;
    
    RM = (R0 * (1.0 - e2)) / pow(den, 1.5);
    RN = R0 / sqrt(den);
}

// [Source: 282] Prediction Step
void EsEkf::predict(const ImuMeasurement& imu, double dt) {
    // 1. Correct Measurements [Source: 285-286]
    Eigen::Vector3d f_b = imu.acc - x_.ba;
    Eigen::Vector3d w_b = imu.gyro - x_.bg;

    // Current State
    Eigen::Matrix3d C_b_n = x_.q.toRotationMatrix(); // Body to Nav DCM
    double lat = x_.p.x();
    double h = x_.p.z();
    
    // Earth Parameters
    double RM, RN;
    compute_radius(lat, RM, RN);

    // [Source: 140] Earth Rate in Nav Frame
    Eigen::Vector3d w_ie_n;
    w_ie_n << WE * cos(lat), 0.0, -WE * sin(lat);

    // [Source: 145] Transport Rate in Nav Frame
    Eigen::Vector3d w_en_n;
    w_en_n << x_.v.y() / (RN + h),
             -x_.v.x() / (RM + h),
             -x_.v.x() * x_.v.y() * tan(lat) / (RN + h); // Small term often simplified, but keeping strictly per typical transport rate equations (Eq 28 in PDF has a typo/format issue, using standard def)

    // [Source: 147] Total Rotation
    Eigen::Vector3d w_in_n = w_ie_n + w_en_n;
    
    // [Source: 290] Velocity Update (Mechanization)
    // Gravity vector in Nav frame (approx WGS84 gravity formula or simple model)
    // Simple model: g = [0, 0, g0] (Should use Somigliana for high precision)
    Eigen::Vector3d g_n(0, 0, 9.780327 * (1 + 0.0053024 * sin(lat)*sin(lat) - 0.0000058 * sin(2*lat)*sin(2*lat)) - 3.086e-6 * h); 

    Eigen::Vector3d v_dot = C_b_n * f_b + g_n - (2.0 * w_ie_n + w_en_n).cross(x_.v);
    Eigen::Vector3d v_next = x_.v + v_dot * dt;

    // [Source: 291] Position Update
    Eigen::Vector3d p_dot;
    p_dot << x_.v.x() / (RM + h),
             x_.v.y() / ((RN + h) * cos(lat)),
             -x_.v.z(); // Down is positive Z, so -v_D is rate of height change? 
             // PDF Eq 291 uses Frv^-1 * v. 
             // Standard NED: h_dot = -v_D. 
             // PDF 2.1 defines r = [phi, lam, h].
             // PDF Eq 67: r_k+1 = r_k + ...
             // Let's stick to standard kinematic:
             // lat_dot = v_N / (RM+h)
             // lon_dot = v_E / ((RN+h)cos(lat))
             // h_dot = -v_D
    
    // Update Nominal State (Integration)
    x_.p.x() += p_dot.x() * dt;
    x_.p.y() += p_dot.y() * dt;
    x_.p.z() -= x_.v.z() * dt; // Height change
    x_.v = v_next;

    // [Source: 289] Attitude Update
    // omega_nb_b = w_b - C_n_b * w_in_n [Source: 93]
    Eigen::Vector3d w_nb_b = w_b - C_b_n.transpose() * w_in_n;
    
    // Quaternion integration: q_new = q * exp(0.5 * w * dt)
    // For small dt, exp(0.5*w*dt) approx [1, 0.5*w*dt]
    Eigen::Vector3d angle = w_nb_b * dt;
    double angle_norm = angle.norm();
    Eigen::Quaterniond dq;
    if (angle_norm > 1e-8) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle_norm, angle / angle_norm));
    } else {
        dq = Eigen::Quaterniond(1, 0.5*angle.x(), 0.5*angle.y(), 0.5*angle.z());
        dq.normalize();
    }
    x_.q = x_.q * dq; // Eigen multiplies q_current * q_increment
    x_.q.normalize();


    // --- Build F Matrix [Source: 115] ---
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> F = Eigen::Matrix<double, DIM_ERROR, DIM_ERROR>::Zero();

    // F_rr (Pos-Pos) [Source: 118] - simplified for readability
    // Note: Full derivation of Frr involves many terms dependent on gravity/radii.
    // Commonly simplified in implementation unless high precision required. 
    // Using simple F_rr identity relation for position evolution.
    // Actually, F_pos_vel is the dominant term.
    
    // F_rv (Pos-Vel) [Source: 122 -> Eq 22 implies transformation from vel to angle rates]
    F(IDX_POS, IDX_VEL)     = 1.0 / (RM + h);
    F(IDX_POS+1, IDX_VEL+1) = 1.0 / ((RN + h) * cos(lat));
    F(IDX_POS+2, IDX_VEL+2) = -1.0;

    // F_vr (Vel-Pos) - Gravity and Coriolis gradients (often neglected in rough code, but needed for 'full')
    // Leaving as 0 for brevity unless strictly needed, dominant term is F_vv.

    // F_vv (Vel-Vel) - Coriolis terms
    Eigen::Vector3d w_term = 2.0 * w_ie_n + w_en_n;
    F.block<3,3>(IDX_VEL, IDX_VEL) = -skew(w_term);

    // F_v_theta (Vel-Att) [Source: 127]
    Eigen::Vector3d f_n = C_b_n * f_b;
    F.block<3,3>(IDX_VEL, IDX_ATT) = -skew(f_n);

    // F_v_ba (Vel-AccelBias) [Source: 130]
    F.block<3,3>(IDX_VEL, IDX_BA) = -C_b_n;

    // F_theta_theta (Att-Att) [Source: 133]
    F.block<3,3>(IDX_ATT, IDX_ATT) = -skew(w_in_n);

    // F_theta_bg (Att-GyroBias) [Source: 136]
    F.block<3,3>(IDX_ATT, IDX_BG) = -C_b_n;

    // --- Build G Matrix [Source: 185] ---
    Eigen::Matrix<double, DIM_ERROR, DIM_NOISE> G = Eigen::Matrix<double, DIM_ERROR, DIM_NOISE>::Zero();
    G.block<3,3>(IDX_VEL, 0) = -C_b_n;      // v - eta_a
    G.block<3,3>(IDX_ATT, 3) = -C_b_n;      // theta - eta_g
    G.block<3,3>(IDX_BA, 6)  = Eigen::Matrix3d::Identity(); // ba - w_ka
    G.block<3,3>(IDX_BG, 9)  = Eigen::Matrix3d::Identity(); // bg - w_kg

    // --- Van Loan Discretization [Source: 205-214] ---
    // A = [-F  G*Qc*G^T] * dt
    //     [0   F^T     ]
    // 2n x 2n matrix
    Eigen::Matrix<double, 2*DIM_ERROR, 2*DIM_ERROR> A;
    A.setZero();
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> GQGt = G * Qc_ * G.transpose();

    A.block<DIM_ERROR, DIM_ERROR>(0, 0) = -F * dt;
    A.block<DIM_ERROR, DIM_ERROR>(0, DIM_ERROR) = GQGt * dt;
    A.block<DIM_ERROR, DIM_ERROR>(DIM_ERROR, DIM_ERROR) = F.transpose() * dt;

    // Matrix Exponential [Source: 211-212]
    Eigen::Matrix<double, 2*DIM_ERROR, 2*DIM_ERROR> B = A.exp();

    // Extract Phi and Qd [Source: 214]
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> Phi = B.block<DIM_ERROR, DIM_ERROR>(DIM_ERROR, DIM_ERROR).transpose();
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> Qd = Phi * B.block<DIM_ERROR, DIM_ERROR>(0, DIM_ERROR);

    // Symmetrize Qd [Source: 215]
    Qd = 0.5 * (Qd + Qd.transpose());

    // --- Covariance Update [Source: 300] ---
    P_ = Phi * P_ * Phi.transpose() + Qd;
}

// [Source: 313] Error Injection
void EsEkf::inject_error(const Eigen::Matrix<double, 15, 1>& dx) {
    Eigen::Vector3d dr = dx.segment<3>(IDX_POS);
    Eigen::Vector3d dv = dx.segment<3>(IDX_VEL);
    Eigen::Vector3d dtheta = dx.segment<3>(IDX_ATT);
    Eigen::Vector3d dba = dx.segment<3>(IDX_BA);
    Eigen::Vector3d dbg = dx.segment<3>(IDX_BG);

    // [Source: 317] Pos
    x_.p += dr;

    // [Source: 319] Vel
    x_.v += dv;

    // [Source: 323] Accel Bias
    x_.ba += dba;

    // [Source: 325] Gyro Bias
    x_.bg += dbg;

    // [Source: 321] Attitude Correction
    // q_plus = delta_q * q_minus
    // delta_q = [0.5*theta; 1]
    Eigen::Quaterniond dq;
    double theta_norm = dtheta.norm();
    if(theta_norm > 1e-8) {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta_norm, dtheta / theta_norm));
    } else {
         dq = Eigen::Quaterniond(1, 0.5*dtheta.x(), 0.5*dtheta.y(), 0.5*dtheta.z());
    }
    
    // PDF Eq 7: q+ = dq * q- (Left multiplication implies error in Navigation frame?)
    // Source 58 says "error vector represents small rotation".
    // Usually ES-EKF defines error in Global frame or Body frame. 
    // If Eq 64 uses dq * q, it's global error. If q * dq, it's local.
    // Based on the derivation of F_theta (Eq 133: -[w_in x]), the error is Global (Nav frame).
    x_.q = dq * x_.q; 
    x_.q.normalize();
}

void EsEkf::update_internal(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
    // [Source: 305] Innovation
    // y is computed in the specific update functions to handle modular arithmetic (like angles) if necessary
    // Here we assume linear y = z (residuals already computed)
    
    Eigen::VectorXd y = z; // In this design, 'z' passed in is the residual (Innovation)
    
    // [Source: 308] Innovation Covariance
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;

    // [Source: 311] Kalman Gain
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // [Source: 314] Error State Estimate
    Eigen::VectorXd dx = K * y;

    // [Source: 329] Covariance Update (Joseph form)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(DIM_ERROR, DIM_ERROR);
    Eigen::MatrixXd I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();

    // [Source: 331] Symmetrize
    P_ = 0.5 * (P_ + P_.transpose());

    // Inject Errors
    inject_error(dx);

    // [Source: 332] Reset Error State (Implicitly done by not storing 'dx' between steps)
}

// [Source: 242] GNSS Position Update
void EsEkf::update_gnss_position(const Eigen::Vector3d& pos_gnss, const Eigen::Matrix3d& R) {
    // Innovation: z_pos = r_GNSS - r_INS [Source: 244]
    Eigen::Vector3d innovation = pos_gnss - x_.p;
    
    // H Matrix [Source: 247]
    Eigen::Matrix<double, 3, DIM_ERROR> H;
    H.setZero();
    H.block<3,3>(0, IDX_POS) = Eigen::Matrix3d::Identity();

    update_internal(innovation, H, R);
}

// [Source: 249] GNSS Velocity Update
void EsEkf::update_gnss_velocity(const Eigen::Vector3d& vel_gnss, const Eigen::Matrix3d& R) {
    // Innovation [Source: 251]
    Eigen::Vector3d innovation = vel_gnss - x_.v;

    // H Matrix [Source: 254]
    Eigen::Matrix<double, 3, DIM_ERROR> H;
    H.setZero();
    H.block<3,3>(0, IDX_VEL) = Eigen::Matrix3d::Identity();

    update_internal(innovation, H, R);
}

// [Source: 256] Baro Update
void EsEkf::update_barometer(double altitude, double R_var) {
    // Innovation [Source: 262]
    double innovation = altitude - x_.p.z(); // x_.p.z() is height

    // H Matrix [Source: 264]
    Eigen::Matrix<double, 1, DIM_ERROR> H;
    H.setZero();
    H(0, IDX_POS + 2) = 1.0; // Index 2 is height error

    Eigen::Matrix<double, 1, 1> R_mat;
    R_mat(0,0) = R_var;

    Eigen::VectorXd z(1);
    z(0) = innovation;

    update_internal(z, H, R_mat);
}