#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "eskf.h"
#include "sensor_io.h"
#include <cmath>
#include <random>

class ESKFUnitTest : public ::testing::Test {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~ESKFUnitTest() = default;
    
protected:
    void SetUp() override {
        params.N_acc = 0.01;
        params.N_gyro = 0.001;
        params.B_acc = 0.005;
        params.B_gyro = 0.0001;
        params.tau_acc = 100.0;
        params.tau_gyro = 100.0;
        init_lla = Eigen::Vector3d(40.7128 * M_PI / 180.0, -74.0060 * M_PI / 180.0, 100.0);
        eskf = std::make_unique<ESKF>(params, init_lla);
    }

    SensorParams params;
    Eigen::Vector3d init_lla;
    std::unique_ptr<ESKF> eskf;
    
    // Helper to check quaternion validity
    bool isValidQuaternion(const Eigen::Quaterniond& q) {
        return std::isfinite(q.w()) && std::isfinite(q.x()) && 
               std::isfinite(q.y()) && std::isfinite(q.z()) &&
               std::abs(q.norm() - 1.0) < 1e-6;
    }
    
    // Helper to check covariance validity
    bool isValidCovariance(const Eigen::MatrixXd& P) {
        if (P.rows() != 15 || P.cols() != 15) return false;
        if (!P.allFinite()) return false;
        if (!P.isApprox(P.transpose(), 1e-9)) return false;
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
        return (es.eigenvalues().array() >= -1e-9).all();
    }
};

// =============================================================================
// INITIALIZATION TESTS
// =============================================================================

TEST_F(ESKFUnitTest, Init_PositionMatchesInput) {
    // Given: ESKF constructed with specific LLA
    // When: We query position
    // Then: It matches exactly
    Eigen::Vector3d pos = eskf->getPositionLLA();
    EXPECT_DOUBLE_EQ(pos(0), init_lla(0));
    EXPECT_DOUBLE_EQ(pos(1), init_lla(1));
    EXPECT_DOUBLE_EQ(pos(2), init_lla(2));
}

TEST_F(ESKFUnitTest, Init_VelocityIsZero) {
    // Given: Fresh ESKF
    // When: We query velocity
    // Then: All components are exactly zero
    Eigen::Vector3d vel = eskf->getVelocity();
    EXPECT_DOUBLE_EQ(vel(0), 0.0);
    EXPECT_DOUBLE_EQ(vel(1), 0.0);
    EXPECT_DOUBLE_EQ(vel(2), 0.0);
}

TEST_F(ESKFUnitTest, Init_AttitudeIsIdentity) {
    // Given: Fresh ESKF
    // When: We query attitude
    // Then: It's identity quaternion
    Eigen::Quaterniond att = eskf->getAttitude();
    EXPECT_NEAR(att.w(), 1.0, 1e-9);
    EXPECT_NEAR(att.x(), 0.0, 1e-9);
    EXPECT_NEAR(att.y(), 0.0, 1e-9);
    EXPECT_NEAR(att.z(), 0.0, 1e-9);
}

TEST_F(ESKFUnitTest, Init_BiasesAreZero) {
    // Given: Fresh ESKF
    // When: We query biases
    // Then: All are exactly zero
    EXPECT_EQ(eskf->getAccelBias().norm(), 0.0);
    EXPECT_EQ(eskf->getGyroBias().norm(), 0.0);
}

TEST_F(ESKFUnitTest, Init_CovarianceIsValid) {
    // Given: Fresh ESKF
    // When: We query covariance
    // Then: It's 15x15, symmetric, positive semi-definite
    EXPECT_TRUE(isValidCovariance(eskf->getCovariance()));
}

TEST_F(ESKFUnitTest, Init_CovarianceIsDiagonal) {
    // Given: Fresh ESKF
    // When: We query covariance
    // Then: Off-diagonal elements are zero (no initial correlations)
    auto P = eskf->getCovariance();
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            if (i != j) {
                EXPECT_DOUBLE_EQ(P(i, j), 0.0);
            }
        }
    }
}

// =============================================================================
// PREDICT - QUATERNION INVARIANTS
// =============================================================================

TEST_F(ESKFUnitTest, Predict_QuaternionRemainsNormalized) {
    // Given: Any prediction inputs
    // When: We predict
    // Then: Quaternion remains unit length
    std::mt19937 gen(42);
    std::uniform_real_distribution<> acc_dist(-2.0, 2.0);
    std::uniform_real_distribution<> gyro_dist(-1.0, 1.0);
    
    for (int i = 0; i < 100; ++i) {
        Eigen::Vector3d acc(acc_dist(gen), acc_dist(gen), acc_dist(gen));
        Eigen::Vector3d gyro(gyro_dist(gen), gyro_dist(gen), gyro_dist(gen));
        eskf->predict(acc, gyro, 0.01);
        ASSERT_TRUE(isValidQuaternion(eskf->getAttitude())) << "Failed at step " << i;
    }
}

TEST_F(ESKFUnitTest, Predict_ZeroGyroPreservesAttitude) {
    // Given: Zero gyro input
    // When: We predict
    // Then: Attitude changes minimally (only due to transport/earth rate)
    Eigen::Quaterniond initial = eskf->getAttitude();
    Eigen::Vector3d zero_gyro(0.0, 0.0, 0.0);
    Eigen::Vector3d any_acc(0.0, 0.0, 0.0);
    
    eskf->predict(any_acc, zero_gyro, 0.01);
    
    Eigen::Quaterniond after = eskf->getAttitude();
    double angle_change = initial.angularDistance(after);
    EXPECT_LT(angle_change, 0.001); // Very small change from earth/transport rate
}

TEST_F(ESKFUnitTest, Predict_GyroProducesRotation) {
    // Given: Non-zero gyro about Z axis
    // When: We predict for multiple steps
    // Then: Attitude changes (yaw increases)
    Eigen::Quaterniond initial = eskf->getAttitude();
    Eigen::Vector3d gyro(0.0, 0.0, 0.5); // 0.5 rad/s about Z
    Eigen::Vector3d any_acc(0.0, 0.0, 0.0);
    
    for (int i = 0; i < 100; ++i) {
        eskf->predict(any_acc, gyro, 0.01);
    }
    
    Eigen::Quaterniond after = eskf->getAttitude();
    double angle_change = initial.angularDistance(after);
    EXPECT_GT(angle_change, 0.1); // Significant rotation occurred
}

// =============================================================================
// PREDICT - COVARIANCE INVARIANTS
// =============================================================================

TEST_F(ESKFUnitTest, Predict_CovarianceRemainsValid) {
    // Given: Any prediction inputs
    // When: We predict
    // Then: Covariance stays symmetric and PSD
    std::mt19937 gen(42);
    std::uniform_real_distribution<> acc_dist(-2.0, 2.0);
    std::uniform_real_distribution<> gyro_dist(-1.0, 1.0);
    
    for (int i = 0; i < 100; ++i) {
        Eigen::Vector3d acc(acc_dist(gen), acc_dist(gen), acc_dist(gen));
        Eigen::Vector3d gyro(gyro_dist(gen), gyro_dist(gen), gyro_dist(gen));
        eskf->predict(acc, gyro, 0.01);
        ASSERT_TRUE(isValidCovariance(eskf->getCovariance())) << "Failed at step " << i;
    }
}

TEST_F(ESKFUnitTest, Predict_CovarianceGrows) {
    // Given: Initial covariance
    // When: We predict without updates
    // Then: Trace increases (uncertainty grows)
    double initial_trace = eskf->getCovariance().trace();
    
    for (int i = 0; i < 100; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.01);
    }
    
    double final_trace = eskf->getCovariance().trace();
    EXPECT_GT(final_trace, initial_trace);
}

// =============================================================================
// PREDICT - VELOCITY MECHANICS
// =============================================================================

TEST_F(ESKFUnitTest, Predict_AccelXChangesVelocityX) {
    // Given: Positive X acceleration
    // When: We predict
    // Then: X velocity changes (direction depends on frame conventions)
    double initial_vx = eskf->getVelocity()(0);
    Eigen::Vector3d acc(1.0, 0.0, 0.0);
    
    for (int i = 0; i < 100; ++i) {
        eskf->predict(acc, Eigen::Vector3d::Zero(), 0.01);
    }
    
    double final_vx = eskf->getVelocity()(0);
    EXPECT_NE(final_vx, initial_vx); // Velocity changed
}

TEST_F(ESKFUnitTest, Predict_AccelYChangesVelocityY) {
    // Given: Positive Y acceleration
    // When: We predict
    // Then: Y velocity changes
    double initial_vy = eskf->getVelocity()(1);
    Eigen::Vector3d acc(0.0, 1.0, 0.0);
    
    for (int i = 0; i < 100; ++i) {
        eskf->predict(acc, Eigen::Vector3d::Zero(), 0.01);
    }
    
    double final_vy = eskf->getVelocity()(1);
    EXPECT_NE(final_vy, initial_vy);
}

TEST_F(ESKFUnitTest, Predict_AccelZChangesVelocityZ) {
    // Given: Positive Z acceleration
    // When: We predict
    // Then: Z velocity changes
    double initial_vz = eskf->getVelocity()(2);
    Eigen::Vector3d acc(0.0, 0.0, 1.0);
    
    for (int i = 0; i < 100; ++i) {
        eskf->predict(acc, Eigen::Vector3d::Zero(), 0.01);
    }
    
    double final_vz = eskf->getVelocity()(2);
    EXPECT_NE(final_vz, initial_vz);
}

TEST_F(ESKFUnitTest, Predict_ZeroInputChangesVelocity) {
    // Given: Zero accelerometer reading (freefall or gravity effect)
    // When: We predict
    // Then: Velocity changes due to gravity model
    double initial_vz = eskf->getVelocity()(2);
    
    for (int i = 0; i < 100; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.01);
    }
    
    double final_vz = eskf->getVelocity()(2);
    EXPECT_NE(final_vz, initial_vz); // Gravity causes change
}

// =============================================================================
// PREDICT - POSITION MECHANICS
// =============================================================================

TEST_F(ESKFUnitTest, Predict_VelocityIntegratesToPosition) {
    // Given: Non-zero velocity (induced by acceleration)
    // When: We continue predicting
    // Then: Position changes
    Eigen::Vector3d initial_pos = eskf->getPositionLLA();
    
    // Induce some velocity
    for (int i = 0; i < 50; ++i) {
        eskf->predict(Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::Zero(), 0.01);
    }
    
    Eigen::Vector3d mid_pos = eskf->getPositionLLA();
    
    // Continue with zero accel (coast)
    for (int i = 0; i < 50; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.01);
    }
    
    Eigen::Vector3d final_pos = eskf->getPositionLLA();
    
    // Position should have changed from initial
    EXPECT_NE(final_pos(0), initial_pos(0));
}

// =============================================================================
// PREDICT - NUMERICAL STABILITY
// =============================================================================

TEST_F(ESKFUnitTest, Predict_HandlesVerySmallDt) {
    // Given: Very small timestep
    // When: We predict
    // Then: All outputs remain finite
    for (int i = 0; i < 100; ++i) {
        eskf->predict(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.01, 0.01, 0.01), 1e-6);
    }
    
    EXPECT_TRUE(eskf->getPositionLLA().allFinite());
    EXPECT_TRUE(eskf->getVelocity().allFinite());
    EXPECT_TRUE(isValidQuaternion(eskf->getAttitude()));
    EXPECT_TRUE(isValidCovariance(eskf->getCovariance()));
}

TEST_F(ESKFUnitTest, Predict_HandlesLargeDt) {
    // Given: Large timestep (edge case)
    // When: We predict
    // Then: All outputs remain finite
    eskf->predict(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.01, 0.01, 0.01), 0.5);
    
    EXPECT_TRUE(eskf->getPositionLLA().allFinite());
    EXPECT_TRUE(eskf->getVelocity().allFinite());
    EXPECT_TRUE(isValidQuaternion(eskf->getAttitude()));
    EXPECT_TRUE(isValidCovariance(eskf->getCovariance()));
}

TEST_F(ESKFUnitTest, Predict_HandlesZeroGyroMagnitude) {
    // Given: Exactly zero gyro (edge case for normalization)
    // When: We predict
    // Then: No crash, valid output
    eskf->predict(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.0, 0.0, 0.0), 0.01);
    EXPECT_TRUE(isValidQuaternion(eskf->getAttitude()));
}

TEST_F(ESKFUnitTest, Predict_HandlesTinyGyro) {
    // Given: Near-zero gyro (numerical precision edge case)
    // When: We predict
    // Then: No crash, valid output
    eskf->predict(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(1e-15, 1e-15, 1e-15), 0.01);
    EXPECT_TRUE(isValidQuaternion(eskf->getAttitude()));
}

TEST_F(ESKFUnitTest, Predict_HandlesLargeAccel) {
    // Given: Large acceleration values
    // When: We predict
    // Then: All outputs remain finite
    eskf->predict(Eigen::Vector3d(10.0, 10.0, 10.0), Eigen::Vector3d::Zero(), 0.01);
    
    EXPECT_TRUE(eskf->getPositionLLA().allFinite());
    EXPECT_TRUE(eskf->getVelocity().allFinite());
}

TEST_F(ESKFUnitTest, Predict_HandlesLargeGyro) {
    // Given: Large gyro values
    // When: We predict
    // Then: All outputs remain finite, quaternion valid
    eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d(5.0, 5.0, 5.0), 0.01);
    
    EXPECT_TRUE(isValidQuaternion(eskf->getAttitude()));
}

// =============================================================================
// PREDICT - GEOGRAPHIC EDGE CASES
// =============================================================================

TEST_F(ESKFUnitTest, Predict_StableAtEquator) {
    // Given: ESKF at equator (lat=0, cos(lat)=1, tan(lat)=0)
    // When: We predict
    // Then: All outputs remain finite
    Eigen::Vector3d equator(0.0, 0.0, 100.0);
    ESKF eq_eskf(params, equator);
    
    for (int i = 0; i < 100; ++i) {
        eq_eskf.predict(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.01, 0.01, 0.01), 0.01);
    }
    
    EXPECT_TRUE(eq_eskf.getPositionLLA().allFinite());
    EXPECT_TRUE(eq_eskf.getVelocity().allFinite());
}

TEST_F(ESKFUnitTest, Predict_StableAtHighLatitude) {
    // Given: ESKF near pole (large tan(lat), small cos(lat))
    // When: We predict
    // Then: All outputs remain finite
    Eigen::Vector3d arctic(85.0 * M_PI / 180.0, 0.0, 100.0);
    ESKF arctic_eskf(params, arctic);
    
    for (int i = 0; i < 100; ++i) {
        arctic_eskf.predict(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.01, 0.01, 0.01), 0.01);
    }
    
    EXPECT_TRUE(arctic_eskf.getPositionLLA().allFinite());
    EXPECT_TRUE(arctic_eskf.getVelocity().allFinite());
}

TEST_F(ESKFUnitTest, Predict_StableAtHighAltitude) {
    // Given: ESKF at high altitude
    // When: We predict
    // Then: All outputs remain finite
    Eigen::Vector3d high_alt(40.0 * M_PI / 180.0, 0.0, 50000.0);
    ESKF high_eskf(params, high_alt);
    
    for (int i = 0; i < 100; ++i) {
        high_eskf.predict(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d(0.01, 0.01, 0.01), 0.01);
    }
    
    EXPECT_TRUE(high_eskf.getPositionLLA().allFinite());
    EXPECT_TRUE(high_eskf.getVelocity().allFinite());
}

// =============================================================================
// BARO UPDATE - MECHANICS
// =============================================================================

TEST_F(ESKFUnitTest, BaroUpdate_ChangesAltitude) {
    // Given: ESKF with some altitude
    // When: We apply baro update with different altitude
    // Then: Altitude state changes
    double initial_alt = eskf->getPositionLLA()(2);
    
    // Pressure corresponding to ~200m (different from init 100m)
    double pressure = 1013.25 * pow(1.0 - 200.0 / 44330.0, 1.0 / 0.1903);
    eskf->updateBaro(pressure);
    
    double final_alt = eskf->getPositionLLA()(2);
    EXPECT_NE(final_alt, initial_alt);
}

TEST_F(ESKFUnitTest, BaroUpdate_DoesNotChangeLat) {
    // Given: ESKF with some position
    // When: We apply baro update
    // Then: Latitude unchanged (H matrix is zero for lat)
    double initial_lat = eskf->getPositionLLA()(0);
    
    eskf->updateBaro(1000.0);
    
    double final_lat = eskf->getPositionLLA()(0);
    EXPECT_DOUBLE_EQ(final_lat, initial_lat);
}

TEST_F(ESKFUnitTest, BaroUpdate_DoesNotChangeLon) {
    // Given: ESKF with some position
    // When: We apply baro update
    // Then: Longitude unchanged
    double initial_lon = eskf->getPositionLLA()(1);
    
    eskf->updateBaro(1000.0);
    
    double final_lon = eskf->getPositionLLA()(1);
    EXPECT_DOUBLE_EQ(final_lon, initial_lon);
}

TEST_F(ESKFUnitTest, BaroUpdate_ReducesAltitudeCovariance) {
    // Given: ESKF after predictions (covariance grown)
    for (int i = 0; i < 100; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.01);
    }
    double cov_alt_before = eskf->getCovariance()(2, 2);
    
    // When: We apply baro update
    eskf->updateBaro(1013.0);
    
    // Then: Altitude covariance decreases
    double cov_alt_after = eskf->getCovariance()(2, 2);
    EXPECT_LT(cov_alt_after, cov_alt_before);
}

TEST_F(ESKFUnitTest, BaroUpdate_CovarianceRemainsValid) {
    // Given: ESKF after predictions
    for (int i = 0; i < 100; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.01);
    }
    
    // When: We apply baro update
    eskf->updateBaro(1013.0);
    
    // Then: Covariance still valid
    EXPECT_TRUE(isValidCovariance(eskf->getCovariance()));
}

TEST_F(ESKFUnitTest, BaroUpdate_QuaternionRemainsValid) {
    // Given: ESKF with some attitude
    for (int i = 0; i < 50; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.1, 0.1), 0.01);
    }
    
    // When: We apply baro update
    eskf->updateBaro(1000.0);
    
    // Then: Quaternion still normalized
    EXPECT_TRUE(isValidQuaternion(eskf->getAttitude()));
}

// =============================================================================
// MAG UPDATE - MECHANICS
// =============================================================================

TEST_F(ESKFUnitTest, MagUpdate_ChangesAttitude) {
    // Given: ESKF with some yaw
    for (int i = 0; i < 100; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 0.1), 0.01);
    }
    Eigen::Quaterniond att_before = eskf->getAttitude();
    
    // When: We apply mag update
    eskf->updateMag(Eigen::Vector3d(1.0, 0.0, -0.5));
    
    // Then: Attitude changed
    Eigen::Quaterniond att_after = eskf->getAttitude();
    EXPECT_GT(att_before.angularDistance(att_after), 1e-6);
}

TEST_F(ESKFUnitTest, MagUpdate_DoesNotChangePosition) {
    // Given: ESKF with some position
    Eigen::Vector3d initial_pos = eskf->getPositionLLA();
    
    // When: We apply mag update
    eskf->updateMag(Eigen::Vector3d(1.0, 0.0, -0.5));
    
    // Then: Position unchanged
    Eigen::Vector3d final_pos = eskf->getPositionLLA();
    EXPECT_DOUBLE_EQ(final_pos(0), initial_pos(0));
    EXPECT_DOUBLE_EQ(final_pos(1), initial_pos(1));
    EXPECT_DOUBLE_EQ(final_pos(2), initial_pos(2));
}

TEST_F(ESKFUnitTest, MagUpdate_VelocityChangeIsSmall) {
    // Given: ESKF with some velocity
    // Note: Mag H matrix only directly observes yaw, but velocity can change
    //       through off-diagonal covariance coupling (this is correct EKF behavior)
    for (int i = 0; i < 50; ++i) {
        eskf->predict(Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::Zero(), 0.01);
    }
    Eigen::Vector3d initial_vel = eskf->getVelocity();
    
    // When: We apply mag update
    eskf->updateMag(Eigen::Vector3d(1.0, 0.0, -0.5));
    
    // Then: Velocity change should be small (only from covariance coupling)
    Eigen::Vector3d final_vel = eskf->getVelocity();
    double vel_change = (final_vel - initial_vel).norm();
    
    // Change should be much smaller than the velocity magnitude itself
    EXPECT_LT(vel_change, initial_vel.norm() * 0.5);
}

TEST_F(ESKFUnitTest, MagUpdate_ReducesYawCovariance) {
    // Given: ESKF after predictions
    for (int i = 0; i < 100; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.01);
    }
    double cov_yaw_before = eskf->getCovariance()(8, 8);
    
    // When: We apply mag update
    eskf->updateMag(Eigen::Vector3d(1.0, 0.0, -0.5));
    
    // Then: Yaw covariance decreases
    double cov_yaw_after = eskf->getCovariance()(8, 8);
    EXPECT_LT(cov_yaw_after, cov_yaw_before);
}

TEST_F(ESKFUnitTest, MagUpdate_CovarianceRemainsValid) {
    // Given: ESKF after predictions
    for (int i = 0; i < 100; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.01);
    }
    
    // When: We apply mag update
    eskf->updateMag(Eigen::Vector3d(1.0, 0.0, -0.5));
    
    // Then: Covariance still valid
    EXPECT_TRUE(isValidCovariance(eskf->getCovariance()));
}

TEST_F(ESKFUnitTest, MagUpdate_QuaternionRemainsNormalized) {
    // Given: ESKF with arbitrary attitude
    for (int i = 0; i < 50; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.2, 0.3), 0.01);
    }
    
    // When: We apply mag update
    eskf->updateMag(Eigen::Vector3d(0.5, 0.2, -0.4));
    
    // Then: Quaternion still normalized
    EXPECT_TRUE(isValidQuaternion(eskf->getAttitude()));
}

// =============================================================================
// COMBINED PREDICT/UPDATE - LONG RUNNING STABILITY
// =============================================================================

TEST_F(ESKFUnitTest, Combined_RemainsStableOver1000Steps) {
    // Given: Random inputs
    std::mt19937 gen(12345);
    std::uniform_real_distribution<> acc_dist(-1.0, 1.0);
    std::uniform_real_distribution<> gyro_dist(-0.5, 0.5);
    
    // When: We run many predict/update cycles
    for (int i = 0; i < 1000; ++i) {
        Eigen::Vector3d acc(acc_dist(gen), acc_dist(gen), acc_dist(gen));
        Eigen::Vector3d gyro(gyro_dist(gen), gyro_dist(gen), gyro_dist(gen));
        
        eskf->predict(acc, gyro, 0.01);
        
        if (i % 10 == 0) eskf->updateBaro(1013.0 + acc_dist(gen));
        if (i % 20 == 0) eskf->updateMag(Eigen::Vector3d(1.0, acc_dist(gen) * 0.1, -0.5));
        
        // Then: All invariants hold at every step
        ASSERT_TRUE(eskf->getPositionLLA().allFinite()) << "Position invalid at step " << i;
        ASSERT_TRUE(eskf->getVelocity().allFinite()) << "Velocity invalid at step " << i;
        ASSERT_TRUE(isValidQuaternion(eskf->getAttitude())) << "Quaternion invalid at step " << i;
        ASSERT_TRUE(isValidCovariance(eskf->getCovariance())) << "Covariance invalid at step " << i;
    }
}

// =============================================================================
// BIAS STATE MECHANICS
// =============================================================================

TEST_F(ESKFUnitTest, Bias_AccelBiasRemainsFinite) {
    // Given: Many prediction steps
    // When: We predict
    // Then: Accel bias remains finite
    for (int i = 0; i < 500; ++i) {
        eskf->predict(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector3d::Zero(), 0.01);
    }
    EXPECT_TRUE(eskf->getAccelBias().allFinite());
}

TEST_F(ESKFUnitTest, Bias_GyroBiasRemainsFinite) {
    // Given: Many prediction steps
    // When: We predict
    // Then: Gyro bias remains finite
    for (int i = 0; i < 500; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.1, 0.1), 0.01);
    }
    EXPECT_TRUE(eskf->getGyroBias().allFinite());
}

TEST_F(ESKFUnitTest, Bias_CovarianceDecayModelWorks) {
    // Given: Initial bias covariance
    double initial_cov_ba = eskf->getCovariance()(9, 9);
    
    // When: We predict (bias states have time constant decay)
    for (int i = 0; i < 100; ++i) {
        eskf->predict(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.01);
    }
    
    // Then: Bias covariance changed (exact behavior depends on Q/F)
    double final_cov_ba = eskf->getCovariance()(9, 9);
    EXPECT_NE(final_cov_ba, initial_cov_ba);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
