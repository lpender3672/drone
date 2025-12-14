#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "../eskf.h"
#include <cmath>

class ESKFTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up sensor parameters for testing
        params.N_acc = 0.01;    // Accelerometer noise
        params.N_gyro = 0.001;  // Gyroscope noise
        params.B_acc = 0.005;   // Accelerometer bias
        params.B_gyro = 0.0001; // Gyroscope bias
        params.tau_acc = 100.0; // Accelerometer correlation time
        params.tau_gyro = 100.0; // Gyroscope correlation time

        // Initial position (LLA) - NYC coordinates
        init_lla = Eigen::Vector3d(40.7128 * M_PI / 180.0, -74.0060 * M_PI / 180.0, 10.0);
        
        // Create ESKF instance
        eskf = std::make_unique<ESKF>(params, init_lla);
    }

    SensorParams params;
    Eigen::Vector3d init_lla;
    std::unique_ptr<ESKF> eskf;
};

// Simple initialization test
TEST_F(ESKFTest, Initialization) {
    EXPECT_TRUE(eskf != nullptr);
    
    // Check initial velocity is zero
    Eigen::Vector3d velocity = eskf->getVelocity();
    EXPECT_NEAR(velocity(0), 0.0, 1e-6);
    EXPECT_NEAR(velocity(1), 0.0, 1e-6);
    EXPECT_NEAR(velocity(2), 0.0, 1e-6);
}

// Simple prediction test  
TEST_F(ESKFTest, PredictStationary) {
    // Get initial state
    Eigen::Vector3d pos_before = eskf->getPositionLLA();
    Eigen::Vector3d vel_before = eskf->getVelocity();
    Eigen::Quaterniond q_before = eskf->getAttitude();
    
    // Zero IMU readings (stationary)
    Eigen::Vector3d acc(0, 0, 0);
    Eigen::Vector3d gyro(0, 0, 0);
    double dt = 0.01;
    
    eskf->predict(acc, gyro, dt);
    
    // Check state after prediction
    Eigen::Vector3d pos_after = eskf->getPositionLLA();
    Eigen::Vector3d vel_after = eskf->getVelocity();
    Eigen::Quaterniond q_after = eskf->getAttitude();
    
    // Position should be approximately same (small gravity effect expected)
    EXPECT_NEAR((pos_after - pos_before).norm(), 0.0, 0.01);
    // Attitude should be unchanged for zero gyro
    EXPECT_NEAR(q_after.coeffs().dot(q_before.coeffs()), 1.0, 1e-6);
}

// Test gravity prediction
TEST_F(ESKFTest, PredictWithGravity) {
    // Get initial state
    Eigen::Vector3d vel_before = eskf->getVelocity();
    
    // Zero acceleration (free fall), zero rotation
    Eigen::Vector3d acc(0, 0, 0);
    Eigen::Vector3d gyro(0, 0, 0);
    double dt = 0.1;
    
    eskf->predict(acc, gyro, dt);
    
    Eigen::Vector3d vel_after = eskf->getVelocity();
    
    // Should have gained downward velocity due to gravity
    // Note: Gravity is in NED frame, so Z is down (+Z)
    EXPECT_GT(vel_after(2), vel_before(2)); // More positive Z velocity = downward
}

// Test attitude rotation prediction
TEST_F(ESKFTest, PredictAttitudeRotation) {
    // Get initial attitude
    Eigen::Quaterniond q_before = eskf->getAttitude();
    
    // Constant angular velocity around Z axis
    Eigen::Vector3d acc(0, 0, 0);
    Eigen::Vector3d gyro(0, 0, 0.1); // 0.1 rad/s around Z
    double dt = 0.1;
    
    eskf->predict(acc, gyro, dt);
    
    Eigen::Quaterniond q_after = eskf->getAttitude();
    
    // Calculate expected rotation
    Eigen::AngleAxisd expected_rotation(0.1 * dt, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_expected = expected_rotation * q_before;
    
    // Check rotation is approximately correct
    double dot_product = q_after.coeffs().dot(q_expected.coeffs());
    EXPECT_NEAR(std::abs(dot_product), 1.0, 0.01);
}
