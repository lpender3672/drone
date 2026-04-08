#include <gtest/gtest.h>

#include "quadcopter/dynamics.hpp"
#include "quadcopter/controller.hpp"
#include "quadcopter/motor.hpp"

using namespace sim;
using namespace sim::quadcopter;

// ============================================================================
// Helpers
// ============================================================================

static QuadrotorDynamics::Params make_test_params() {
    QuadrotorDynamics::Params p;
    p.mass             = 0.5;
    p.arm_length       = 0.175;
    p.inertia          = Vec3(0.0035, 0.0035, 0.0055);
    p.motor.tau        = 0.02;
    p.motor.omega_max  = 2500.0;
    p.propeller.k_t    = 3.27e-7;
    p.propeller.k_q    = 4.25e-9;
    p.propeller.d      = 1.0;
    p.propeller.rho    = 1.225;
    return p;
}

static TrueState make_initial_state(double z = -5.0) {
    TrueState s;
    s.position         = Vec3(0.0, 0.0, z);
    s.velocity         = Vec3::Zero();
    s.attitude         = Quat::Identity();
    s.angular_velocity = Vec3::Zero();
    return s;
}

static MotorEfforts uniform_efforts(double throttle) {
    MotorEfforts e;
    e[0] = e[1] = e[2] = e[3] = throttle;
    return e;
}

// Advance dynamics by duration_ms milliseconds, setting efforts each step.
// t_us is updated in place.
static void step_dynamics(QuadrotorDynamics* dyn, const MotorEfforts& efforts,
                          uint64_t& t_us, uint64_t duration_ms) {
    uint64_t end = t_us + duration_ms * 1000;
    while (t_us < end) {
        dyn->input().set(efforts);
        dyn->update(t_us);
        t_us += 1000;
    }
}

// ============================================================================
// Dynamics tests
// ============================================================================

class DynamicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        dyn = std::make_unique<QuadrotorDynamics>("dyn", 1000, 100);
        dyn->set_params(make_test_params());
        dyn->reset(make_initial_state(-5.0));
        t_us = 0;
    }

    std::unique_ptr<QuadrotorDynamics> dyn;
    uint64_t t_us;
};

TEST_F(DynamicsTest, InitialState) {
    const auto& s = dyn->output().get();
    EXPECT_DOUBLE_EQ(s.position.z(), -5.0);
    EXPECT_DOUBLE_EQ(s.velocity.x(),  0.0);
    EXPECT_DOUBLE_EQ(s.velocity.z(),  0.0);
    EXPECT_NEAR(s.attitude.w(), 1.0, 1e-9);
}

TEST_F(DynamicsTest, FreeFall) {
    // Zero throttle: motors produce no thrust, drone falls (NED: +z is down)
    step_dynamics(dyn.get(), uniform_efforts(0.0), t_us, 500);

    const auto& s = dyn->output().get();
    EXPECT_GT(s.velocity.z(), 3.0);    // falling at several m/s
    EXPECT_GT(s.position.z(), -5.0);   // has moved downward (z increased in NED)
}

TEST_F(DynamicsTest, SymmetricThrottleNoTorque) {
    // Equal throttle on all motors → no roll or pitch torque
    step_dynamics(dyn.get(), uniform_efforts(0.5), t_us, 300);

    const auto& s = dyn->output().get();
    EXPECT_NEAR(s.angular_velocity.x(), 0.0, 1e-6);
    EXPECT_NEAR(s.angular_velocity.y(), 0.0, 1e-6);
}

TEST_F(DynamicsTest, DifferentialThrottleRollTorque) {
    // Motors 0,3 higher than 1,2 → tau_roll = L*(T0+T3-T1-T2) > 0 → positive roll rate
    MotorEfforts e;
    e[0] = 0.7; e[1] = 0.3; e[2] = 0.3; e[3] = 0.7;
    step_dynamics(dyn.get(), e, t_us, 300);

    EXPECT_GT(dyn->output().get().angular_velocity.x(), 0.1);
}

TEST_F(DynamicsTest, DifferentialThrottlePitchTorque) {
    // Motors 1,3 higher than 0,2 → tau_pitch = L*(T1+T3-T0-T2) > 0 → positive pitch rate
    MotorEfforts e;
    e[0] = 0.3; e[1] = 0.7; e[2] = 0.3; e[3] = 0.7;
    step_dynamics(dyn.get(), e, t_us, 300);

    EXPECT_GT(dyn->output().get().angular_velocity.y(), 0.1);
}

TEST_F(DynamicsTest, GroundCollision) {
    // Start just above ground, drop → position clamped at z=0, velocity zeroed
    dyn->reset(make_initial_state(-0.05));
    step_dynamics(dyn.get(), uniform_efforts(0.0), t_us, 200);

    const auto& s = dyn->output().get();
    EXPECT_NEAR(s.position.z(), 0.0, 0.001);
    EXPECT_LE(s.velocity.z(), 0.0);
}

// ============================================================================
// Controller tests
// ============================================================================

class ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        ctrl = std::make_unique<AttitudePidController>("ctrl", 1000);
    }

    void run(const shared::TrueState& state, const AttitudeReference& ref) {
        shared::ObservedState obs(state);
        ctrl->state_input().set(obs);
        ctrl->reference_input().set(ref);
        ctrl->update(0);
    }

    const MotorEfforts& out() { return ctrl->output().get(); }

    std::unique_ptr<AttitudePidController> ctrl;
};

TEST_F(ControllerTest, LevelSymmetry) {
    // Level state + level reference → all four motor efforts equal
    AttitudeReference ref;
    ref.set_roll(0.0); ref.set_pitch(0.0); ref.set_yaw(0.0);
    ref.set_thrust(0.5);

    run(make_initial_state(), ref);

    EXPECT_NEAR(out()[0], out()[1], 1e-6);
    EXPECT_NEAR(out()[1], out()[2], 1e-6);
    EXPECT_NEAR(out()[2], out()[3], 1e-6);
}

TEST_F(ControllerTest, ThrustPassthrough) {
    // Level attitude, only thrust → each motor gets exactly that thrust value
    AttitudeReference ref;
    ref.set_roll(0.0); ref.set_pitch(0.0); ref.set_yaw(0.0);
    ref.set_thrust(0.7);

    run(make_initial_state(), ref);

    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(out()[i], 0.7, 1e-6);
}

TEST_F(ControllerTest, RollErrorCorrection) {
    // State rolled +0.1 rad, reference level → negative roll rate setpoint →
    // negative roll mixer output → motors 1,2 get more than 0,3.
    // Corrective torque: tau_roll = L*(T0+T3-T1-T2) < 0, reducing positive roll.
    shared::TrueState state;
    state.position         = Vec3::Zero();
    state.velocity         = Vec3::Zero();
    state.angular_velocity = Vec3::Zero();
    state.set_from_euler(0.1, 0.0, 0.0);

    AttitudeReference ref;
    ref.set_roll(0.0); ref.set_pitch(0.0); ref.set_yaw(0.0);
    ref.set_thrust(0.5);

    run(state, ref);

    EXPECT_GT(out()[1], out()[0]);
    EXPECT_GT(out()[2], out()[3]);
    EXPECT_NEAR(out()[1], out()[2], 1e-6);  // symmetric about roll axis
    EXPECT_NEAR(out()[0], out()[3], 1e-6);
}

TEST_F(ControllerTest, PitchErrorCorrection) {
    // State pitched +0.1 rad, reference level → negative pitch rate setpoint →
    // negative pitch mixer output → motors 0,2 get more than 1,3.
    // Corrective torque: tau_pitch = L*(T1+T3-T0-T2) < 0, reducing positive pitch.
    shared::TrueState state;
    state.position         = Vec3::Zero();
    state.velocity         = Vec3::Zero();
    state.angular_velocity = Vec3::Zero();
    state.set_from_euler(0.0, 0.1, 0.0);

    AttitudeReference ref;
    ref.set_roll(0.0); ref.set_pitch(0.0); ref.set_yaw(0.0);
    ref.set_thrust(0.5);

    run(state, ref);

    EXPECT_GT(out()[0], out()[1]);
    EXPECT_GT(out()[2], out()[3]);
    EXPECT_NEAR(out()[0], out()[2], 1e-6);  // symmetric about pitch axis
    EXPECT_NEAR(out()[1], out()[3], 1e-6);
}
