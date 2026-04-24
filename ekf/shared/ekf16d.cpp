#include "ekf16d.h"

void EKF16d::feed_imu(const sensors::ImuMeasurement& imu) {
    double dt = last_imu_dt_;
    if (last_imu_timestamp_us_ > 0 && imu.timestamp_us > last_imu_timestamp_us_) {
        dt = (imu.timestamp_us - last_imu_timestamp_us_) * 1e-6;
    }
    last_imu_timestamp_us_ = imu.timestamp_us;
    last_imu_dt_ = dt;

    // Cache bias-corrected angular velocity for output()
    last_omega_ = imu.gyro - bg();

    predict(imu, dt);

    // Gravity aiding at 100 Hz: use the accelerometer to directly constrain roll/pitch.
    // Only fire when the vehicle is approximately in 1-g flight (not hard maneuvering).
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

    // EKF pos state is (lat_rad, lon_rad, alt_m); pass geodetic directly.
    const double lat_rad = gnss.latitude_deg  * (M_PI / 180.0);
    const double lon_rad = gnss.longitude_deg * (M_PI / 180.0);
    Eigen::Vector3d pos_geo(lat_rad, lon_rad, gnss.altitude_m);

    // Horizontal accuracy: prefer receiver-reported 1-sigma if available, else derive from HDOP.
    // Scale from m to rad using meridian (RM) and normal (RN) radii of curvature.
    double RM, RN;
    compute_radius<double>(lat_rad, RM, RN);
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

    state.position = pos();
    state.velocity = vel();

    // Quaternion stored in state as [w,x,y,z]; Eigen::Quaterniond ctor is (w,x,y,z).
    Eigen::Vector4d qv = q_vec();
    state.attitude = Eigen::Quaterniond(qv(0), qv(1), qv(2), qv(3));

    state.angular_velocity = last_omega_;
    state.accel_bias       = ba();
    state.gyro_bias        = bg();
    state.baro_bias        = bbaro();

    return state;
}

void EKF16d::reset(const shared::StateWithBiases& initial) {
    NominalVector x0;
    x0.setZero();

    x0.segment<3>(NOM_POS) = initial.position;
    x0.segment<3>(NOM_VEL) = initial.velocity;

    x0(NOM_QUAT + 0) = initial.attitude.w();
    x0(NOM_QUAT + 1) = initial.attitude.x();
    x0(NOM_QUAT + 2) = initial.attitude.y();
    x0(NOM_QUAT + 3) = initial.attitude.z();

    x0.segment<3>(NOM_BA) = initial.accel_bias;
    x0.segment<3>(NOM_BG) = initial.gyro_bias;
    x0(NOM_BBARO)        = initial.baro_bias;

    CovMatrix P0;
    P0.setIdentity();
    P0.block<3,3>(ERR_POS, ERR_POS) *= 10.0;
    P0.block<3,3>(ERR_VEL, ERR_VEL) *= 1.0;
    P0.block<3,3>(ERR_ATT, ERR_ATT) *= 0.1;
    P0.block<3,3>(ERR_BA, ERR_BA)   *= 0.01;
    P0.block<3,3>(ERR_BG, ERR_BG)   *= 0.001;
    P0(ERR_BBARO, ERR_BBARO)         = 1.0;

    initialize(x0, P0);

    last_omega_            = initial.angular_velocity;
    last_imu_timestamp_us_ = 0;
}
