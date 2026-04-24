#ifndef EKF16F_H
#define EKF16F_H

#include "ekf16d.h"  // for DIM_NOMINAL/ERR/NOISE and enum indices

// Float-scalar variant of the 16-error-state EKF. Intended for STM32 targets
// with single-precision FPU where double would fall back to software float.
//
// Scaffold only: this class inherits the templated EKF base but does not
// yet implement predict/update methods. The implementation port from
// ekf16d.cpp to float is separate follow-up work — it requires either
// moving the implementation to a header or explicit-instantiating both
// scalar types, and revalidating covariance math at lower precision.
class EKF16f : public EKF<float, DIM_NOMINAL, DIM_ERROR, DIM_NOISE> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EKF16f(const EkfErrorParameters& p)
        : EKF<float, DIM_NOMINAL, DIM_ERROR, DIM_NOISE>(p) {}
};

#endif // EKF16F_H
