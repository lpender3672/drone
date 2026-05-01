#include <gtest/gtest.h>
#include <memory>
#include "blocks/quadrotor_ekf.hpp"
#include "mocks/mock_observer.h"

namespace {

// Convenience: construct an EKF block with a mock observer and return
// raw pointers for both. The block keeps ownership of the observer.
struct BlockAndObserver {
    std::unique_ptr<shared::QuadrotorEkfBlock> block;
    mocks::MockObserver* obs = nullptr;
};

BlockAndObserver make_block(uint32_t period_us = 1000) {
    auto observer = std::make_unique<mocks::MockObserver>();
    auto* obs_raw = observer.get();
    auto block = std::make_unique<shared::QuadrotorEkfBlock>(
        "ekf", std::move(observer), period_us);
    return {std::move(block), obs_raw};
}

sensors::ImuMeasurement valid_imu(uint64_t ts_us = 1) {
    sensors::ImuMeasurement m;
    m.acc = sensors::Vec3(0.0, 0.0, 9.81);  // squaredNorm > 1
    m.timestamp_us = ts_us;
    return m;
}

}  // namespace

TEST(QuadrotorEkfBlock, ImuFedOnceWhenTimestampHoldsSteady) {
    auto [ekf, obs] = make_block();

    ekf->imu_input().set(valid_imu(100));
    ekf->update(1000);
    EXPECT_EQ(obs->imu_count, 1);

    // Same timestamp on a later tick — gate must skip.
    ekf->update(2000);
    EXPECT_EQ(obs->imu_count, 1);

    // New timestamp — gate fires.
    ekf->imu_input().set(valid_imu(200));
    ekf->update(3000);
    EXPECT_EQ(obs->imu_count, 2);
}

TEST(QuadrotorEkfBlock, ImuRejectedWhenAccelerationTooSmall) {
    auto [ekf, obs] = make_block();

    sensors::ImuMeasurement m;
    m.acc = sensors::Vec3(0.1, 0.1, 0.1);  // squaredNorm = 0.03 < 1.0
    m.timestamp_us = 100;
    ekf->imu_input().set(m);
    ekf->update(1000);
    EXPECT_EQ(obs->imu_count, 0);
}

TEST(QuadrotorEkfBlock, GnssFedOnlyWhenTimestampAdvances) {
    auto [ekf, obs] = make_block();
    ekf->imu_input().set(valid_imu());  // satisfy IMU path

    sensors::GnssMeasurement g;
    g.timestamp_us = 100;
    ekf->gnss_input().set(g);
    ekf->update(1000);
    EXPECT_EQ(obs->gnss_count, 1);

    ekf->update(2000);  // same timestamp
    EXPECT_EQ(obs->gnss_count, 1);

    g.timestamp_us = 200;
    ekf->gnss_input().set(g);
    ekf->update(3000);
    EXPECT_EQ(obs->gnss_count, 2);
}

TEST(QuadrotorEkfBlock, GnssNotFedWhenInputUnconnected) {
    auto [ekf, obs] = make_block();
    ekf->imu_input().set(valid_imu());
    // Deliberately do not touch gnss_input — it stays unconnected.
    ekf->update(1000);
    EXPECT_EQ(obs->gnss_count, 0);
}

TEST(QuadrotorEkfBlock, GnssNotFedWhenDisabled) {
    auto [ekf, obs] = make_block();
    ekf->set_gnss_enabled(false);
    ekf->imu_input().set(valid_imu());

    sensors::GnssMeasurement g;
    g.timestamp_us = 100;
    ekf->gnss_input().set(g);
    ekf->update(1000);
    EXPECT_EQ(obs->gnss_count, 0);
}

TEST(QuadrotorEkfBlock, BaroFedOnlyWhenTimestampAdvances) {
    auto [ekf, obs] = make_block();
    ekf->imu_input().set(valid_imu());

    sensors::BaroMeasurement b;
    b.timestamp_us = 100;
    ekf->baro_input().set(b);
    ekf->update(1000);
    EXPECT_EQ(obs->baro_count, 1);

    ekf->update(2000);
    EXPECT_EQ(obs->baro_count, 1);

    b.timestamp_us = 200;
    ekf->baro_input().set(b);
    ekf->update(3000);
    EXPECT_EQ(obs->baro_count, 2);
}

TEST(QuadrotorEkfBlock, MagFedOnlyWhenTimestampAdvances) {
    auto [ekf, obs] = make_block();
    ekf->imu_input().set(valid_imu());

    sensors::MagMeasurement m;
    m.timestamp_us = 100;
    ekf->mag_input().set(m);
    ekf->update(1000);
    EXPECT_EQ(obs->mag_count, 1);

    ekf->update(2000);
    EXPECT_EQ(obs->mag_count, 1);

    m.timestamp_us = 200;
    ekf->mag_input().set(m);
    ekf->update(3000);
    EXPECT_EQ(obs->mag_count, 2);
}

TEST(QuadrotorEkfBlock, OriginRoundTrips) {
    auto [ekf, obs] = make_block();
    shared::GnssOrigin in;
    in.lat_deg = 1.0; in.lon_deg = 2.0; in.alt_m = 3.0;
    ekf->set_origin(in);
    EXPECT_DOUBLE_EQ(ekf->origin().lat_deg, 1.0);
    EXPECT_DOUBLE_EQ(ekf->origin().lon_deg, 2.0);
    EXPECT_DOUBLE_EQ(ekf->origin().alt_m,   3.0);
}

TEST(QuadrotorEkfBlock, InitializeResetsObserverWithGeodeticState) {
    auto [ekf, obs] = make_block();
    shared::GnssOrigin origin;
    origin.lat_deg = 0.0; origin.lon_deg = 0.0; origin.alt_m = 100.0;
    ekf->set_origin(origin);

    shared::TrueState init;
    init.position = shared::Vec3(0.0, 0.0, -5.0);  // 5 m above origin (NED z-down)
    ekf->initialize(init);

    EXPECT_EQ(obs->reset_count, 1);
    // Geodetic altitude should be origin.alt_m - position.z() = 100 - (-5) = 105.
    EXPECT_DOUBLE_EQ(obs->last_reset.position.z(), 105.0);
}

TEST(QuadrotorEkfBlock, RespectsUpdatePeriod) {
    auto [ekf, obs] = make_block(/*period_us=*/5000);
    ekf->imu_input().set(valid_imu(1));

    EXPECT_TRUE(ekf->update(0));         // first call always due
    EXPECT_FALSE(ekf->update(1000));     // dt < period
    EXPECT_TRUE(ekf->update(5000));      // dt == period
}
