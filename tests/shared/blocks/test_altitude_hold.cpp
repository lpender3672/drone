#include <gtest/gtest.h>
#include <algorithm>
#include "blocks/altitude_hold.hpp"

using shared::AltitudeHoldBlock;
using shared::NavigationState;

namespace {

NavigationState state_at(double pos_z, double vel_z = 0.0) {
    NavigationState s;
    s.position.z() = pos_z;
    s.velocity.z() = vel_z;
    return s;
}

// Default params plus tweaks. The block exposes its Params via .params() but
// doesn't let you mutate it in place, so this helper rebuilds a copy.
AltitudeHoldBlock<NavigationState>::Params with_gains(double kp, double ki, double kd) {
    AltitudeHoldBlock<NavigationState>::Params p;
    p.kp = kp; p.ki = ki; p.kd = kd;
    return p;
}

}  // namespace

TEST(AltitudeHoldBlock, OutputAtHoverWhenAtSetpointAndZeroVelocity) {
    AltitudeHoldBlock<NavigationState> ah("ah");
    ah.set_setpoint_m(10.0);
    ah.input().set(state_at(10.0, 0.0));
    ah.update(0);
    EXPECT_DOUBLE_EQ(ah.output().get().value(), ah.params().hover_thrust);
}

TEST(AltitudeHoldBlock, OutputAboveHoverWhenBelowSetpoint) {
    AltitudeHoldBlock<NavigationState> ah("ah");
    ah.set_setpoint_m(10.0);
    ah.input().set(state_at(8.0, 0.0));   // 2 m below → P pushes thrust up
    ah.update(0);
    EXPECT_GT(ah.output().get().value(), ah.params().hover_thrust);
}

TEST(AltitudeHoldBlock, OutputBelowHoverWhenAboveSetpoint) {
    AltitudeHoldBlock<NavigationState> ah("ah");
    ah.set_setpoint_m(10.0);
    ah.input().set(state_at(12.0, 0.0));
    ah.update(0);
    EXPECT_LT(ah.output().get().value(), ah.params().hover_thrust);
}

TEST(AltitudeHoldBlock, OutputClampsAtUpperBound) {
    AltitudeHoldBlock<NavigationState> ah("ah");
    auto p = ah.params();  // start from defaults
    p.kp = 10.0; p.ki = 0.0; p.kd = 0.0;
    ah.set_params(p);

    ah.set_setpoint_m(100.0);
    ah.input().set(state_at(0.0, 0.0));   // huge error → P term saturates
    ah.update(0);
    EXPECT_DOUBLE_EQ(ah.output().get().value(),
                     ah.params().hover_thrust + ah.params().out_clamp);
}

TEST(AltitudeHoldBlock, OutputClampsAtLowerBound) {
    AltitudeHoldBlock<NavigationState> ah("ah");
    auto p = ah.params();
    p.kp = 10.0; p.ki = 0.0; p.kd = 0.0;
    ah.set_params(p);

    ah.set_setpoint_m(0.0);
    ah.input().set(state_at(100.0, 0.0));
    ah.update(0);
    EXPECT_DOUBLE_EQ(ah.output().get().value(),
                     ah.params().hover_thrust - ah.params().out_clamp);
}

TEST(AltitudeHoldBlock, FirstTickSkipsIntegral) {
    AltitudeHoldBlock<NavigationState> ah("ah");
    ah.set_params(with_gains(/*kp=*/0.0, /*ki=*/1.0, /*kd=*/0.0));

    ah.set_setpoint_m(10.0);
    ah.input().set(state_at(0.0, 0.0));   // error = 10 m
    ah.update(0);
    // Without P or D contribution, output should be the hover baseline —
    // the integral term must not have accumulated on the very first tick.
    EXPECT_DOUBLE_EQ(ah.output().get().value(), ah.params().hover_thrust);
}

TEST(AltitudeHoldBlock, IntegralAccumulatesAfterFirstTick) {
    AltitudeHoldBlock<NavigationState> ah("ah");
    auto p = ah.params();
    p.kp = 0.0; p.ki = 0.01; p.kd = 0.0;
    p.int_clamp = 100.0;  // raise so the integral isn't capped during the test
    ah.set_params(p);

    ah.set_setpoint_m(10.0);
    ah.input().set(state_at(0.0, 0.0));   // error = 10 m
    ah.update(0);
    ah.update(1'000'000);                 // dt = 1 s, integral += 10 * 1 = 10
    // ki * integral = 0.01 * 10 = 0.10 above hover.
    EXPECT_NEAR(ah.output().get().value(), ah.params().hover_thrust + 0.10, 1e-9);
}

TEST(AltitudeHoldBlock, AntiWindupHaltsIntegralWhenSaturated) {
    AltitudeHoldBlock<NavigationState> ah("ah");
    auto p = ah.params();
    p.kp = 10.0;   // huge P guarantees saturation
    p.ki = 1.0;    // would otherwise wind up massively
    p.kd = 0.0;
    ah.set_params(p);

    ah.set_setpoint_m(100.0);
    ah.input().set(state_at(0.0, 0.0));
    for (uint64_t t = 0; t <= 10'000'000; t += 1'000'000) {
        ah.update(t);
    }
    // Pull the integral by reading at-setpoint output: integral should be ≈ 0
    // (anti-windup blocked accumulation throughout). At setpoint with zero
    // velocity, output should be hover_thrust + ki*integral. If integral were
    // anywhere near unbounded, this test would catch it.
    ah.input().set(state_at(100.0, 0.0));
    ah.update(11'000'000);
    EXPECT_NEAR(ah.output().get().value(), ah.params().hover_thrust,
                ah.params().ki * 1.0);  // tolerance: at most 1 s of accumulated err
}

TEST(AltitudeHoldBlock, ResetClearsIntegralAndFirstTickState) {
    AltitudeHoldBlock<NavigationState> ah("ah");
    ah.set_params(with_gains(0.0, 1.0, 0.0));

    ah.set_setpoint_m(10.0);
    ah.input().set(state_at(0.0, 0.0));
    ah.update(0);
    ah.update(1'000'000);  // integral builds

    ah.reset();
    ah.update(2'000'000);  // first tick after reset — integral skipped
    EXPECT_DOUBLE_EQ(ah.output().get().value(), ah.params().hover_thrust);
}
