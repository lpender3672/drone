#ifndef EKF16F_H
#define EKF16F_H

#include "ekf16.h"

// Float-scalar 16-error-state EKF. Intended for STM32 targets with
// single-precision FPU where double would fall back to software float.
// No IObserver adapter — float flight stacks provide their own integration.
using EKF16f = EKF16<float>;

#endif // EKF16F_H
