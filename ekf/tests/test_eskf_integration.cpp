#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "../eskf.h"
#include <cmath>
#include <random>

// Simulates sensor data for controlled test scenarios
class MockSensorData {
public:
    MockSensorData(unsigned seed = 42) : gen_(seed) {}
    
    Eigen::Vector3d noisyAccel(const Eigen::Vector3d& true_val, double std_dev) {
        std::normal_distribution<> d(0.0, std_dev);
        return true_val + Eigen::Vector3d(d(gen_), d(gen_), d(gen_));
    }
    
    Eigen::Vector3d noisyGyro(const Eigen::Vector3d& true_val, double std_dev) {
        std::normal_distribution<> d(0.0, std_dev);
        return true_val + Eigen::Vector3d(d(gen_), d(gen_), d(gen_));
    }
    
    double noisyBaro(double true_alt, double std_dev) {
        std::normal_distribution<> d(0.0, std_dev);
        double noisy_alt = true_alt + d(gen_);
        return 1013.25 * pow(1.0 - noisy_alt / 44330.0, 1.0 / 0.1903);
    }

private:
    std::mt19937 gen_;
};

class ESKFIntegrationTest : public ::testing::Test {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~ESKFIntegrationTest() = default;
    
protected:
    void SetUp() override {
        params.N_acc = 0.01;
        params.N_gyro = 0.001;
        params.B_acc = 0.005;
        params.B_gyro = 0.0001;
        params.tau_acc = 100.0;
        params.tau_gyro = 100.0;
        init_lla = Eigen::Vector3d(39.7392 * M_PI / 180.0, -104.9903 * M_PI / 180.0, 1609.0);
    }
    
    bool isValidState(ESKF& eskf) {
        if (!eskf.getPositionLLA().allFinite()) return false;
        if (!eskf.getVelocity().allFinite()) return false;
        auto q = eskf.getAttitude();
        if (!std::isfinite(q.norm()) || std::abs(q.norm() - 1.0) > 1e-6) return false;
        auto P = eskf.getCovariance();
        if (!P.allFinite()) return false;
        if (!P.isApprox(P.transpose(), 1e-9)) return false;
        return true;
    }

    SensorParams params;
    Eigen::Vector3d init_lla;
};

// =============================================================================
// EXTENDED DURATION - NUMERICAL STABILITY
// =============================================================================

TEST_F(ESKFIntegrationTest, StableFor60SecondsWithConstantInput) {
    // Given: ESKF with constant sensor inputs for 60 seconds
    ESKF eskf(params, init_lla);
    double dt = 0.01;
    int steps = 6000;
    
    Eigen::Vector3d const_accel(0.0, 0.0, 0.0);
    Eigen::Vector3d const_gyro(0.0, 0.0, 0.0);
    
    // When: We run for 60 seconds
    for (int i = 0; i < steps; ++i) {
        eskf.predict(const_accel, const_gyro, dt);
        
        // Then: State remains valid at each step
        ASSERT_TRUE(isValidState(eskf)) << "Invalid state at step " << i;
    }
}

TEST_F(ESKFIntegrationTest, StableFor60SecondsWithNoisyInput) {
    // Given: ESKF with noisy sensor inputs
    ESKF eskf(params, init_lla);
    MockSensorData mock(42);
    double dt = 0.01;
    int steps = 6000;
    
    Eigen::Vector3d base_accel(0.0, 0.0, 0.0);
    Eigen::Vector3d base_gyro(0.0, 0.0, 0.0);
    
    // When: We run for 60 seconds with noise
    for (int i = 0; i < steps; ++i) {
        Eigen::Vector3d noisy_accel = mock.noisyAccel(base_accel, 0.05);
        Eigen::Vector3d noisy_gyro = mock.noisyGyro(base_gyro, 0.01);
        
        eskf.predict(noisy_accel, noisy_gyro, dt);
        
        // Then: State remains valid
        ASSERT_TRUE(isValidState(eskf)) << "Invalid state at step " << i;
    }
}

TEST_F(ESKFIntegrationTest, StableFor5MinutesWithUpdates) {
    // Given: ESKF with predictions and periodic updates
    ESKF eskf(params, init_lla);
    MockSensorData mock(42);
    double dt = 0.01;
    int steps = 30000; // 5 minutes
    
    // When: We run for 5 minutes with baro updates
    for (int i = 0; i < steps; ++i) {
        Eigen::Vector3d noisy_accel = mock.noisyAccel(Eigen::Vector3d::Zero(), 0.02);
        Eigen::Vector3d noisy_gyro = mock.noisyGyro(Eigen::Vector3d::Zero(), 0.005);
        
        eskf.predict(noisy_accel, noisy_gyro, dt);
        
        if (i % 20 == 0) {
            eskf.updateBaro(mock.noisyBaro(init_lla(2), 2.0));
        }
        if (i % 50 == 0) {
            eskf.updateMag(Eigen::Vector3d(1.0, 0.0, -0.5));
        }
        
        // Then: State remains valid
        ASSERT_TRUE(isValidState(eskf)) << "Invalid state at step " << i;
    }
}

// =============================================================================
// COVARIANCE BEHAVIOR OVER TIME
// =============================================================================

TEST_F(ESKFIntegrationTest, CovarianceGrowsWithoutUpdates) {
    // Given: ESKF running without measurement updates
    ESKF eskf(params, init_lla);
    double dt = 0.01;
    
    double initial_trace = eskf.getCovariance().trace();
    
    // When: We predict for 10 seconds without updates
    for (int i = 0; i < 1000; ++i) {
        eskf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
    }
    
    // Then: Covariance trace should be larger
    double final_trace = eskf.getCovariance().trace();
    EXPECT_GT(final_trace, initial_trace);
}

TEST_F(ESKFIntegrationTest, CovarianceBoundedWithUpdates) {
    // Given: ESKF with regular updates
    ESKF eskf(params, init_lla);
    MockSensorData mock(42);
    double dt = 0.01;
    
    std::vector<double> traces;
    
    // When: We run for 30 seconds with regular updates
    for (int i = 0; i < 3000; ++i) {
        eskf.predict(mock.noisyAccel(Eigen::Vector3d::Zero(), 0.01), 
                     mock.noisyGyro(Eigen::Vector3d::Zero(), 0.001), dt);
        
        if (i % 10 == 0) {
            eskf.updateBaro(mock.noisyBaro(init_lla(2), 1.0));
        }
        
        if (i % 100 == 0) {
            traces.push_back(eskf.getCovariance().trace());
        }
    }
    
    // Then: Trace should stabilize (not grow unbounded)
    // Check that later traces don't keep growing exponentially
    double mid_trace = traces[traces.size() / 2];
    double final_trace = traces.back();
    
    // Final should not be much larger than middle
    EXPECT_LT(final_trace, mid_trace * 10.0);
}

TEST_F(ESKFIntegrationTest, CovarianceRecoverAfterOutage) {
    // Given: ESKF running normally
    ESKF eskf(params, init_lla);
    MockSensorData mock(42);
    double dt = 0.01;
    
    // Normal operation for 10 seconds
    for (int i = 0; i < 1000; ++i) {
        eskf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
        if (i % 10 == 0) eskf.updateBaro(mock.noisyBaro(init_lla(2), 1.0));
    }
    double normal_trace = eskf.getCovariance().trace();
    
    // When: 30 second measurement outage
    for (int i = 0; i < 3000; ++i) {
        eskf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
    }
    double outage_trace = eskf.getCovariance().trace();
    
    // Then: Covariance should have grown during outage
    EXPECT_GT(outage_trace, normal_trace);
    
    // When: Updates resume
    for (int i = 0; i < 1000; ++i) {
        eskf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
        if (i % 10 == 0) eskf.updateBaro(mock.noisyBaro(init_lla(2), 1.0));
    }
    double recovery_trace = eskf.getCovariance().trace();
    
    // Then: Covariance should decrease after updates resume
    EXPECT_LT(recovery_trace, outage_trace);
}

// =============================================================================
// ROTATION ACCUMULATION
// =============================================================================

TEST_F(ESKFIntegrationTest, GyroInputAccumulatesRotation) {
    // Given: Constant gyro input about Z axis
    ESKF eskf(params, init_lla);
    double omega = 0.1; // rad/s
    Eigen::Vector3d gyro(0.0, 0.0, omega);
    double dt = 0.01;
    double duration = 10.0;
    
    Eigen::Quaterniond initial_att = eskf.getAttitude();
    
    // When: We integrate for 10 seconds
    int steps = static_cast<int>(duration / dt);
    for (int i = 0; i < steps; ++i) {
        eskf.predict(Eigen::Vector3d::Zero(), gyro, dt);
    }
    
    // Then: Total rotation should be non-trivial
    Eigen::Quaterniond final_att = eskf.getAttitude();
    double total_rotation = initial_att.angularDistance(final_att);
    
    // Should have rotated significantly (exact amount depends on implementation)
    EXPECT_GT(total_rotation, 0.5); // At least 0.5 rad
}

TEST_F(ESKFIntegrationTest, OppositeGyroReturnsToStart) {
    // Given: Rotation in one direction then opposite
    ESKF eskf(params, init_lla);
    double omega = 0.2;
    double dt = 0.01;
    int half_steps = 500;
    
    Eigen::Quaterniond initial_att = eskf.getAttitude();
    
    // When: Rotate CW then CCW
    for (int i = 0; i < half_steps; ++i) {
        eskf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, omega), dt);
    }
    Eigen::Quaterniond mid_att = eskf.getAttitude();
    
    for (int i = 0; i < half_steps; ++i) {
        eskf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -omega), dt);
    }
    Eigen::Quaterniond final_att = eskf.getAttitude();
    
    // Then: Should return closer to start than the midpoint
    double dist_mid = initial_att.angularDistance(mid_att);
    double dist_final = initial_att.angularDistance(final_att);
    EXPECT_LT(dist_final, dist_mid);
}

// =============================================================================
// VELOCITY ACCUMULATION
// =============================================================================

TEST_F(ESKFIntegrationTest, AccelInputAccumulatesVelocity) {
    // Given: Constant acceleration input
    ESKF eskf(params, init_lla);
    Eigen::Vector3d accel(0.5, 0.0, 0.0); // 0.5g in X
    double dt = 0.01;
    
    Eigen::Vector3d initial_vel = eskf.getVelocity();
    
    // When: We integrate for 1 second
    for (int i = 0; i < 100; ++i) {
        eskf.predict(accel, Eigen::Vector3d::Zero(), dt);
    }
    
    // Then: Velocity magnitude should have changed
    Eigen::Vector3d final_vel = eskf.getVelocity();
    double vel_change = (final_vel - initial_vel).norm();
    EXPECT_GT(vel_change, 1.0); // Some velocity was accumulated
}

TEST_F(ESKFIntegrationTest, OppositeAccelReducesVelocity) {
    // Given: Accelerate then decelerate
    ESKF eskf(params, init_lla);
    double dt = 0.01;
    int half_steps = 500;
    
    // When: Accelerate forward
    for (int i = 0; i < half_steps; ++i) {
        eskf.predict(Eigen::Vector3d(0.2, 0.0, 0.0), Eigen::Vector3d::Zero(), dt);
    }
    double peak_vel_x = std::abs(eskf.getVelocity()(0));
    
    // When: Decelerate (opposite direction)
    for (int i = 0; i < half_steps; ++i) {
        eskf.predict(Eigen::Vector3d(-0.2, 0.0, 0.0), Eigen::Vector3d::Zero(), dt);
    }
    double final_vel_x = std::abs(eskf.getVelocity()(0));
    
    // Then: Final velocity should be less than peak
    EXPECT_LT(final_vel_x, peak_vel_x);
}

// =============================================================================
// DIFFERENT INITIAL CONDITIONS
// =============================================================================

TEST_F(ESKFIntegrationTest, StableAtMultipleLatitudes) {
    // Given: Different initial latitudes
    std::vector<double> latitudes = {0.0, 30.0, 45.0, 60.0, 80.0}; // degrees
    
    for (double lat_deg : latitudes) {
        double lat_rad = lat_deg * M_PI / 180.0;
        Eigen::Vector3d lla(lat_rad, 0.0, 100.0);
        ESKF eskf(params, lla);
        MockSensorData mock(42);
        
        // When: We run for 30 seconds
        for (int i = 0; i < 3000; ++i) {
            eskf.predict(mock.noisyAccel(Eigen::Vector3d::Zero(), 0.01),
                        mock.noisyGyro(Eigen::Vector3d::Zero(), 0.001), 0.01);
        }
        
        // Then: State remains valid
        EXPECT_TRUE(isValidState(eskf)) << "Failed at latitude " << lat_deg;
    }
}

TEST_F(ESKFIntegrationTest, StableAtMultipleAltitudes) {
    // Given: Different initial altitudes
    std::vector<double> altitudes = {0.0, 1000.0, 10000.0, 30000.0};
    
    for (double alt : altitudes) {
        Eigen::Vector3d lla(40.0 * M_PI / 180.0, 0.0, alt);
        ESKF eskf(params, lla);
        MockSensorData mock(42);
        
        // When: We run for 30 seconds
        for (int i = 0; i < 3000; ++i) {
            eskf.predict(mock.noisyAccel(Eigen::Vector3d::Zero(), 0.01),
                        mock.noisyGyro(Eigen::Vector3d::Zero(), 0.001), 0.01);
        }
        
        // Then: State remains valid
        EXPECT_TRUE(isValidState(eskf)) << "Failed at altitude " << alt;
    }
}

// =============================================================================
// DIFFERENT SENSOR NOISE LEVELS
// =============================================================================

TEST_F(ESKFIntegrationTest, StableWithHighNoise) {
    // Given: High sensor noise
    ESKF eskf(params, init_lla);
    MockSensorData mock(42);
    double dt = 0.01;
    
    // When: We run with high noise
    for (int i = 0; i < 3000; ++i) {
        eskf.predict(mock.noisyAccel(Eigen::Vector3d::Zero(), 0.5),  // High accel noise
                    mock.noisyGyro(Eigen::Vector3d::Zero(), 0.1),    // High gyro noise
                    dt);
        
        // Then: State remains valid
        ASSERT_TRUE(isValidState(eskf)) << "Invalid at step " << i;
    }
}

TEST_F(ESKFIntegrationTest, StableWithVeryLowNoise) {
    // Given: Very low sensor noise
    ESKF eskf(params, init_lla);
    MockSensorData mock(42);
    double dt = 0.01;
    
    // When: We run with minimal noise
    for (int i = 0; i < 3000; ++i) {
        eskf.predict(mock.noisyAccel(Eigen::Vector3d::Zero(), 1e-6),
                    mock.noisyGyro(Eigen::Vector3d::Zero(), 1e-7),
                    dt);
        
        // Then: State remains valid
        ASSERT_TRUE(isValidState(eskf)) << "Invalid at step " << i;
    }
}

// =============================================================================
// UPDATE FREQUENCY VARIATIONS
// =============================================================================

TEST_F(ESKFIntegrationTest, StableWithFrequentUpdates) {
    // Given: Baro update every prediction step
    ESKF eskf(params, init_lla);
    MockSensorData mock(42);
    double dt = 0.01;
    
    // When: Update every step for 30 seconds
    for (int i = 0; i < 3000; ++i) {
        eskf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
        eskf.updateBaro(mock.noisyBaro(init_lla(2), 1.0));
        
        ASSERT_TRUE(isValidState(eskf)) << "Invalid at step " << i;
    }
}

TEST_F(ESKFIntegrationTest, StableWithRareUpdates) {
    // Given: Baro update only every 5 seconds
    ESKF eskf(params, init_lla);
    MockSensorData mock(42);
    double dt = 0.01;
    
    // When: Run with rare updates
    for (int i = 0; i < 6000; ++i) {
        eskf.predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), dt);
        
        if (i % 500 == 0) { // Every 5 seconds
            eskf.updateBaro(mock.noisyBaro(init_lla(2), 1.0));
        }
        
        ASSERT_TRUE(isValidState(eskf)) << "Invalid at step " << i;
    }
}

// =============================================================================
// COMBINED SENSOR UPDATES
// =============================================================================

TEST_F(ESKFIntegrationTest, MultipleUpdatesSameTimestep) {
    // Given: Both baro and mag updates at same time
    ESKF eskf(params, init_lla);
    MockSensorData mock(42);
    double dt = 0.01;
    
    // When: We apply multiple updates per timestep
    for (int i = 0; i < 3000; ++i) {
        eskf.predict(mock.noisyAccel(Eigen::Vector3d::Zero(), 0.01),
                    mock.noisyGyro(Eigen::Vector3d::Zero(), 0.001), dt);
        
        if (i % 10 == 0) {
            eskf.updateBaro(mock.noisyBaro(init_lla(2), 1.0));
            eskf.updateMag(Eigen::Vector3d(1.0, 0.0, -0.5));
        }
        
        ASSERT_TRUE(isValidState(eskf)) << "Invalid at step " << i;
    }
}

