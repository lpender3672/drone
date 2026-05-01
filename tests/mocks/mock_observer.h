#pragma once

#include <observer.h>
#include <sensors/sensor_readings.h>
#include <types/state.h>

namespace mocks {

// INavObserver test double — records every feed_* and reset() call so
// tests can assert what the block-under-test actually pushed through.
// Lives in tests/mocks/ so any future block test can reuse it without
// redefining the same boilerplate.
class MockObserver : public shared::INavObserver {
public:
    int imu_count   = 0;
    int gnss_count  = 0;
    int baro_count  = 0;
    int mag_count   = 0;
    int reset_count = 0;
    sensors::ImuMeasurement  last_imu{};
    sensors::GnssMeasurement last_gnss{};
    sensors::BaroMeasurement last_baro{};
    sensors::MagMeasurement  last_mag{};
    shared::NavigationState  last_reset{};
    shared::NavigationState  canned_output{};

    void feed_imu (const sensors::ImuMeasurement&  m) override { ++imu_count;  last_imu  = m; }
    void feed_gnss(const sensors::GnssMeasurement& m) override { ++gnss_count; last_gnss = m; }
    void feed_baro(const sensors::BaroMeasurement& m) override { ++baro_count; last_baro = m; }
    void feed_mag (const sensors::MagMeasurement&  m) override { ++mag_count;  last_mag  = m; }
    void reset(const shared::NavigationState& s) override { ++reset_count; last_reset = s; }
    shared::NavigationState output() const override { return canned_output; }
};

} // namespace mocks
