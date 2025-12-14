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
        
        eskf = std::make_unique<ESKF>(params, init_lla);
    }

    void TearDown() override {
        eskf.reset();
    }

    SensorParams params;
    Eigen::Vector3d init_lla;
    std::unique_ptr<ESKF> eskf;
};

// Test ESKF initialization
TEST_F(ESKFTest, Initialization) {
    ASSERT_NE(eskf, nullptr);
    
    // Check initial position
    Eigen::Vector3d pos = eskf->getPositionLLA();
    EXPECT_NEAR(pos(0), init_lla(0), 1e-9); // Latitude
    EXPECT_NEAR(pos(1), init_lla(1), 1e-9); // Longitude
    EXPECT_NEAR(pos(2), init_lla(2), 1e-6); // Altitude
    
    // Check initial velocity (should be zero)
    Eigen::Vector3d vel = eskf->getVelocity();
    EXPECT_NEAR(vel.norm(), 0.0, 1e-9);
    
    // Check initial attitude (should be identity quaternion)
    Eigen::Quaterniond att = eskf->getAttitude();
    EXPECT_NEAR(att.w(), 1.0, 1e-6);
    EXPECT_NEAR(att.x(), 0.0, 1e-6);
    EXPECT_NEAR(att.y(), 0.0, 1e-6);
    EXPECT_NEAR(att.z(), 0.0, 1e-6);
}

// Test predict function with zero inputs (stationary case)
TEST_F(ESKFTest, PredictStationary) {
    Eigen::Vector3d zero_accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d zero_gyro = Eigen::Vector3d::Zero();
    double dt = 0.1; // 100ms
    
    Eigen::Vector3d initial_pos = eskf->getPositionLLA();
    Eigen::Vector3d initial_vel = eskf->getVelocity();
    
    eskf->predict(zero_accel, zero_gyro, dt);
    
    Eigen::Vector3d final_pos = eskf->getPositionLLA();
    Eigen::Vector3d final_vel = eskf->getVelocity();
    
    // With zero accelerometer input, the filter should account for gravity
    // Position should change slightly due to gravity
    // Velocity should increase downward due to gravity
    EXPECT_LT(final_vel(2), initial_vel(2)); // Velocity becomes more negative (downward)
}

// Test predict function with gravity compensation
TEST_F(ESKFTest, PredictWithGravity) {
    // Provide 1g upward acceleration to counteract gravity
    Eigen::Vector3d gravity_accel(0.0, 0.0, 1.0); // 1g in body frame (upward)
    Eigen::Vector3d zero_gyro = Eigen::Vector3d::Zero();
    double dt = 0.1;
    
    Eigen::Vector3d initial_vel = eskf->getVelocity();
    
    eskf->predict(gravity_accel, zero_gyro, dt);
    
    Eigen::Vector3d final_vel = eskf->getVelocity();
    
    // With 1g upward acceleration, velocity should remain approximately the same
    // (gravity is compensated)
    EXPECT_NEAR(final_vel(0), initial_vel(0), 0.1);
    EXPECT_NEAR(final_vel(1), initial_vel(1), 0.1);
    EXPECT_NEAR(final_vel(2), initial_vel(2), 0.1);
}

// Test attitude prediction with rotation
TEST_F(ESKFTest, PredictAttitudeRotation) {
    Eigen::Vector3d zero_accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_z(0.0, 0.0, 0.1); // 0.1 rad/s rotation around Z-axis
    double dt = 1.0; // 1 second
    
    Eigen::Quaterniond initial_att = eskf->getAttitude();
    
    eskf->predict(zero_accel, gyro_z, dt);
    
    Eigen::Quaterniond final_att = eskf->getAttitude();
    
    // Calculate expected rotation
    double expected_angle = 0.1 * dt; // 0.1 radians
    Eigen::Quaterniond expected_rotation(Eigen::AngleAxisd(expected_angle, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond expected_att = initial_att * expected_rotation;
    
    // Check if the attitude has rotated approximately as expected
    EXPECT_NEAR(final_att.w(), expected_att.w(), 0.1);
    EXPECT_NEAR(final_att.x(), expected_att.x(), 0.1);
    EXPECT_NEAR(final_att.y(), expected_att.y(), 0.1);
    EXPECT_NEAR(final_att.z(), expected_att.z(), 0.1);
}

// Test multiple prediction steps
TEST_F(ESKFTest, MultiplePredictionSteps) {
    Eigen::Vector3d gravity_accel(0.0, 0.0, 1.0);
    Eigen::Vector3d zero_gyro = Eigen::Vector3d::Zero();
    double dt = 0.01; // 10ms steps
    int num_steps = 100; // Total 1 second
    
    Eigen::Vector3d initial_pos = eskf->getPositionLLA();
    
    for (int i = 0; i < num_steps; ++i) {
        eskf->predict(gravity_accel, zero_gyro, dt);
    }
    
    Eigen::Vector3d final_pos = eskf->getPositionLLA();
    
    // Position should not drift significantly with compensated gravity
    EXPECT_NEAR(final_pos(0), initial_pos(0), 1e-6); // Latitude
    EXPECT_NEAR(final_pos(1), initial_pos(1), 1e-6); // Longitude
    EXPECT_NEAR(final_pos(2), initial_pos(2), 1.0);  // Altitude (allow some drift)
}

// Test magnetometer update
TEST_F(ESKFTest, MagnetometerUpdate) {
    // Earth's magnetic field vector (approximate for NYC)
    Eigen::Vector3d mag_field(0.2, 0.0, -0.5); // Normalized magnetic field
    
    Eigen::Quaterniond initial_att = eskf->getAttitude();
    
    eskf->updateMag(mag_field);
    
    Eigen::Quaterniond final_att = eskf->getAttitude();
    
    // The attitude should be updated based on magnetometer reading
    // This is a basic test to ensure the function doesn't crash and produces reasonable output
    EXPECT_TRUE(std::isfinite(final_att.w()));
    EXPECT_TRUE(std::isfinite(final_att.x()));
    EXPECT_TRUE(std::isfinite(final_att.y()));
    EXPECT_TRUE(std::isfinite(final_att.z()));
    
    // Check quaternion is normalized
    EXPECT_NEAR(final_att.norm(), 1.0, 1e-6);
}

// Test bias estimation
TEST_F(ESKFTest, BiasEstimation) {
    Eigen::Vector3d initial_accel_bias = eskf->getAccelBias();
    Eigen::Vector3d initial_gyro_bias = eskf->getGyroBias();
    
    // Apply consistent bias in measurements
    Eigen::Vector3d biased_accel(0.1, 0.0, 1.0); // With bias
    Eigen::Vector3d biased_gyro(0.01, 0.0, 0.0); // With bias
    double dt = 0.1;
    
    // Run multiple prediction steps
    for (int i = 0; i < 100; ++i) {
        eskf->predict(biased_accel, biased_gyro, dt);
    }
    
    Eigen::Vector3d final_accel_bias = eskf->getAccelBias();
    Eigen::Vector3d final_gyro_bias = eskf->getGyroBias();
    
    // Biases should be finite
    EXPECT_TRUE(std::isfinite(final_accel_bias.norm()));
    EXPECT_TRUE(std::isfinite(final_gyro_bias.norm()));
}

// Test filter stability with noisy inputs
TEST_F(ESKFTest, FilterStability) {
    double dt = 0.01;
    int num_steps = 1000;
    
    // Add some random noise to simulate real sensor data
    std::srand(12345); // Fixed seed for reproducible tests
    
    for (int i = 0; i < num_steps; ++i) {
        // Generate noisy sensor data
        Eigen::Vector3d noisy_accel = Eigen::Vector3d::Random() * 0.1 + Eigen::Vector3d(0, 0, 1.0);
        Eigen::Vector3d noisy_gyro = Eigen::Vector3d::Random() * 0.01;
        
        eskf->predict(noisy_accel, noisy_gyro, dt);
        
        // Check that all outputs remain finite
        Eigen::Vector3d pos = eskf->getPositionLLA();
        Eigen::Vector3d vel = eskf->getVelocity();
        Eigen::Quaterniond att = eskf->getAttitude();
        
        EXPECT_TRUE(std::isfinite(pos.norm()));
        EXPECT_TRUE(std::isfinite(vel.norm()));
        EXPECT_TRUE(std::isfinite(att.norm()));
        
        // Check quaternion remains normalized
        EXPECT_NEAR(att.norm(), 1.0, 1e-3);
    }
}

// Test covariance matrix properties
TEST_F(ESKFTest, CovarianceMatrix) {
    // Get covariance matrix
    Eigen::MatrixXd P = eskf->getCovariance();
    
    // Check dimensions
    EXPECT_EQ(P.rows(), 15);
    EXPECT_EQ(P.cols(), 15);
    
    // Check if matrix is symmetric
    EXPECT_TRUE(P.isApprox(P.transpose(), 1e-9));
    
    // Check if matrix is positive semi-definite (all eigenvalues >= 0)
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
    Eigen::VectorXd eigenvals = es.eigenvalues();
    
    for (int i = 0; i < eigenvals.size(); ++i) {
        EXPECT_GE(eigenvals(i), -1e-9); // Allow small numerical errors
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
