#ifndef EKF16D_H
#define EKF16D_H

#include "ekf16.h"
#include "../../shared/observer.h"
#include "../../shared/types/state.h"

// Double-precision EKF with the IObserver<NavigationState> adapter for
// sim/Teensy/RPi drivers. All predict/update math lives in the templated
// EKF16<double> base (ekf16.cpp).
class EKF16d : public EKF16<double>,
               public shared::IObserver<shared::NavigationState> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EKF16d(const EkfErrorParameters& p) : EKF16<double>(p) {}

    // IObserver interface (high-level sensor feed)
    void feed_imu(const sensors::ImuMeasurement& imu) override;
    void feed_mag(const sensors::MagMeasurement& mag) override;
    void feed_baro(const sensors::BaroMeasurement& baro) override;
    void feed_gnss(const sensors::GnssMeasurement& gnss) override;

    shared::NavigationState output() const override;
    void reset(const shared::NavigationState& initial) override;
};

#endif // EKF16D_H
