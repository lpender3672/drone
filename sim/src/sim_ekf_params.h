#ifndef SIM_EKF_PARAMS_H
#define SIM_EKF_PARAMS_H

#include "ekf.h"

// Sim-only EKF params. Not generated from slab data — the sim uses synthetic
// noise (sensors_impl.hpp) so the Allan-deviation path doesn't apply.
// Values here are hand-tuned to match the sim's sensor noise models:
//   gyro white noise:  0.001 rad/s,  bias walk: 0.0001 rad/s
//   accel white noise: 0.05  m/s²,   bias walk: 0.001  m/s²
//   baro white noise:  0.3   m,      bias walk: 0.01   m
//
// Note on units: the *_n fields are the per-sample std here, not the Allan N
// (which would be per-sample-std / √Fs). The sim generates pure white noise
// at each sample, so this shortcut is OK for sim — but hardware values from
// tune.py are true Allan N coefficients.
inline constexpr EkfErrorParameters SIM_DATA_PARAMS =
{
    .sampling_freq = 1000.0,
    .gyro_x_n  = 1.0e-03,  .gyro_x_b  = 1.0e-04,  .gyro_x_tp  = 100.0,
    .gyro_y_n  = 1.0e-03,  .gyro_y_b  = 1.0e-04,  .gyro_y_tp  = 100.0,
    .gyro_z_n  = 1.0e-03,  .gyro_z_b  = 1.0e-04,  .gyro_z_tp  = 100.0,
    .accel_x_n = 5.0e-02,  .accel_x_b = 1.0e-03,  .accel_x_tp = 100.0,
    .accel_y_n = 5.0e-02,  .accel_y_b = 1.0e-03,  .accel_y_tp = 100.0,
    .accel_z_n = 5.0e-02,  .accel_z_b = 1.0e-03,  .accel_z_tp = 100.0,
    .baro_altitude_n = 3.0e-01,  .baro_altitude_b = 1.0e-02,  .baro_altitude_tp = 10.0,
    // Empirical override — tuned for sim tracking bandwidth, not theoretical minimum.
    .gravity_sigma_x = 3.0e-03,
    .gravity_sigma_y = 3.0e-03,
    .gravity_sigma_z = 3.0e-03,
};

#endif // SIM_EKF_PARAMS_H
