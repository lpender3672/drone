#include <gtest/gtest.h>
#include "blocks/pid.hpp"

using shared::PidBlock;
using shared::PidParams;
using shared::PidInput;

namespace {

PidParams pure_p(double kp) {
    PidParams p;
    p.kp = kp; p.ki = 0.0; p.kd = 0.0;
    p.output_min  = -1000.0; p.output_max  = 1000.0;
    p.integral_min = -1000.0; p.integral_max = 1000.0;
    return p;
}

PidParams pure_i(double ki, double i_clamp = 1000.0) {
    PidParams p;
    p.kp = 0.0; p.ki = ki; p.kd = 0.0;
    p.output_min   = -1000.0; p.output_max   = 1000.0;
    p.integral_min = -i_clamp; p.integral_max = i_clamp;
    return p;
}

PidParams pure_d(double kd) {
    PidParams p;
    p.kp = 0.0; p.ki = 0.0; p.kd = kd;
    p.output_min  = -1000.0; p.output_max  = 1000.0;
    return p;
}

}  // namespace

TEST(PidBlock, OutputZeroBeforeUpdate) {
    PidBlock pid("pid");
    EXPECT_DOUBLE_EQ(pid.output().get().value(), 0.0);
}

TEST(PidBlock, ProportionalSteadyState) {
    PidBlock pid("pid");
    pid.set_params(pure_p(0.5));

    pid.input().set(PidInput{2.0, 0.0});
    pid.update(1000);
    EXPECT_DOUBLE_EQ(pid.output().get().value(), 1.0);  // 0.5 * 2
}

TEST(PidBlock, OutputClampsAtMax) {
    PidBlock pid("pid");
    PidParams p = pure_p(100.0);
    p.output_max = 5.0; p.output_min = -5.0;
    pid.set_params(p);

    pid.input().set(PidInput{10.0, 0.0});  // raw P = 1000
    pid.update(1000);
    EXPECT_DOUBLE_EQ(pid.output().get().value(), 5.0);
}

TEST(PidBlock, OutputClampsAtMin) {
    PidBlock pid("pid");
    PidParams p = pure_p(100.0);
    p.output_max = 5.0; p.output_min = -5.0;
    pid.set_params(p);

    pid.input().set(PidInput{-10.0, 0.0});
    pid.update(1000);
    EXPECT_DOUBLE_EQ(pid.output().get().value(), -5.0);
}

// Regression test for the first-tick bug we fixed: mark_updated() used to
// flip the I/D guards before they could see "first tick", so the integral
// and derivative ran against zero history.
TEST(PidBlock, FirstTickSkipsIntegral) {
    PidBlock pid("pid");
    pid.set_params(pure_i(1.0));

    pid.input().set(PidInput{5.0, 0.0});
    pid.update(1000);
    EXPECT_DOUBLE_EQ(pid.output().get().value(), 0.0);
}

TEST(PidBlock, FirstTickSkipsDerivative) {
    PidBlock pid("pid");
    pid.set_params(pure_d(1.0));

    pid.input().set(PidInput{5.0, 0.0});
    pid.update(1000);
    EXPECT_DOUBLE_EQ(pid.output().get().value(), 0.0);
}

TEST(PidBlock, IntegralAccumulatesAfterFirstTick) {
    PidBlock pid("pid");
    pid.set_params(pure_i(1.0));

    pid.input().set(PidInput{5.0, 0.0});
    pid.update(0);            // first tick — integral skipped
    pid.update(1'000'000);    // dt = 1 s, integral += 5 * 1 = 5
    EXPECT_DOUBLE_EQ(pid.output().get().value(), 5.0);
}

TEST(PidBlock, IntegralClampsToBound) {
    PidBlock pid("pid");
    pid.set_params(pure_i(1.0, /*i_clamp=*/2.5));

    pid.input().set(PidInput{10.0, 0.0});
    for (uint64_t t = 0; t <= 10'000'000; t += 1'000'000) {
        pid.update(t);
    }
    EXPECT_DOUBLE_EQ(pid.output().get().value(), 2.5);
}

TEST(PidBlock, DerivativeFiresOnSetpointChange) {
    PidBlock pid("pid");
    pid.set_params(pure_d(1.0));

    pid.input().set(PidInput{0.0, 0.0});
    pid.update(0);                            // first tick — D skipped, prev_error = 0
    pid.input().set(PidInput{1.0, 0.0});      // error jumps from 0 to 1
    pid.update(1'000'000);                    // dt = 1 s → D = 1
    EXPECT_DOUBLE_EQ(pid.output().get().value(), 1.0);
}

TEST(PidBlock, ResetClearsIntegralAndFirstTickState) {
    PidBlock pid("pid");
    pid.set_params(pure_i(1.0));

    pid.input().set(PidInput{5.0, 0.0});
    pid.update(0);
    pid.update(1'000'000);
    ASSERT_DOUBLE_EQ(pid.output().get().value(), 5.0);

    pid.reset();
    pid.update(2'000'000);  // first tick after reset — integral skipped
    EXPECT_DOUBLE_EQ(pid.output().get().value(), 0.0);
}

TEST(PidBlock, RespectsUpdatePeriod) {
    PidBlock pid("pid", /*update_period_us=*/5000);
    pid.set_params(pure_p(1.0));
    pid.input().set(PidInput{3.0, 0.0});

    EXPECT_TRUE(pid.update(0));         // first call always due
    EXPECT_FALSE(pid.update(1000));     // dt < period
    EXPECT_FALSE(pid.update(4999));     // dt < period
    EXPECT_TRUE(pid.update(5000));      // dt == period
}
