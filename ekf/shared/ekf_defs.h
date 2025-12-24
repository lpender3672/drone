#ifndef EKF_DEFS_H
#define EKF_DEFS_H

#include <Eigen/Dense>

struct EkfStatus {
    bool positive_definite_guaranteed;  // Gershgorin lower bounds all > 0
    bool symmetry_ok;                   // P = P^T within tolerance
    bool diagonal_positive;             // All P(i,i) > 0
    bool variances_bounded;             // No runaway growth
    double min_gershgorin_lower;        // Smallest eigenvalue lower bound
    double max_variance;                // Largest diagonal element
    int suspect_state;                  // Index of most concerning state (-1 if ok)
};

struct ImuMeasurement {
    double t;               // Timestamp [s]
    Eigen::Vector3d acc;    // Specific Force [m/s^2] (already converted from g) [Source: 201]
    Eigen::Vector3d gyro;   // Angular Rate [rad/s]
};

#endif // EKF_DEFS_H