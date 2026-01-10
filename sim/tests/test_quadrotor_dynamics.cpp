#include <gtest/gtest.h>
#include "quadcopter/quadrotor_dynamics.hpp"

namespace sim {
namespace quadcopter {
namespace {

class QuadrotorDynamicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        dynamics_ = std::make_unique<QuadrotorDynamics>("test_dynamics", 1000.0);
        
        State initial;
        initial.position = Vec3(0, 0, -10);  // 10m altitude (NED)
        initial.velocity = Vec3::Zero();
        initial.attitude = Quat::Identity();
        initial.angular_velocity = Vec3::Zero();
        
        dynamics_->reset(initial);
    }
    
    std::unique_ptr<QuadrotorDynamics> dynamics_;
};

TEST_F(QuadrotorDynamicsTest, HoverThrust) {
    // Calculate hover thrust: mg / (4 * max_thrust)
    auto& params = dynamics_->params();
    double hover_effort = (params.mass * params.gravity) / (4.0 * params.max_thrust);
    
    MotorEfforts efforts(hover_effort, hover_effort, hover_effort, hover_effort);
    dynamics_->set_motor_efforts(efforts);
    
    // Run for 1 second
    for (int i = 0; i < 1000; ++i) {
        dynamics_->update(i * 0.001);
    }
    
    const auto& state = dynamics_->true_state();
    
    // Should maintain approximately constant altitude
    EXPECT_NEAR(state.position.z(), -10.0, 0.5);
    EXPECT_NEAR(state.velocity.z(), 0.0, 0.1);
}

TEST_F(QuadrotorDynamicsTest, FreeFall) {
    // Zero thrust - should fall
    MotorEfforts efforts(0.0, 0.0, 0.0, 0.0);
    dynamics_->set_motor_efforts(efforts);
    
    // Run for 0.5 seconds
    for (int i = 0; i < 500; ++i) {
        dynamics_->update(i * 0.001);
    }
    
    const auto& state = dynamics_->true_state();
    
    // Should have fallen and gained downward velocity
    EXPECT_GT(state.position.z(), -10.0);  // NED: positive z is down
    EXPECT_GT(state.velocity.z(), 0.0);    // Downward velocity
}

TEST_F(QuadrotorDynamicsTest, RollTorque) {
    // Asymmetric thrust to induce roll
    // More thrust on right motors (0, 3) should roll left (negative roll)
    auto& params = dynamics_->params();
    double base = (params.mass * params.gravity) / (4.0 * params.max_thrust);
    
    MotorEfforts efforts(base + 0.1, base - 0.1, base - 0.1, base + 0.1);
    dynamics_->set_motor_efforts(efforts);
    
    // Run for 0.2 seconds
    for (int i = 0; i < 200; ++i) {
        dynamics_->update(i * 0.001);
    }
    
    const auto& state = dynamics_->true_state();
    
    // Should have non-zero roll rate
    EXPECT_NE(state.angular_velocity.x(), 0.0);
}

TEST_F(QuadrotorDynamicsTest, InitialState) {
    const auto& state = dynamics_->true_state();
    
    EXPECT_DOUBLE_EQ(state.position.x(), 0.0);
    EXPECT_DOUBLE_EQ(state.position.y(), 0.0);
    EXPECT_DOUBLE_EQ(state.position.z(), -10.0);
}

} // namespace
} // namespace quadcopter
} // namespace sim
