#include <gtest/gtest.h>
#include "ekf16d.h" 
#include "tuned_ekf_params.h"

// Test Fixture for initializing common state
class Ekf16Tests : public ::testing::Test {
protected:
    EKF16d ekf;
    Eigen::Vector3d p0, v0, ba0, bg0;
    double bb0;
    Eigen::Quaterniond q0;
    Eigen::Matrix<double, DIM_ERROR, DIM_ERROR> P0;
    
    // Constants for checking results
    const double g_approx = 9.80665;

    void SetUp() override {
        // [cite: 40] Initialize Nominal State
        p0 = Eigen::Vector3d::Zero(); // Lat/Lon/Alt
        v0 = Eigen::Vector3d::Zero();
        q0 = Eigen::Quaterniond::Identity();
        ba0 = Eigen::Vector3d::Zero();
        bg0 = Eigen::Vector3d::Zero();
        bb0 = 0.0;
        
        EKF16d::CovMatrix P0;
        P0.setIdentity();
        P0.block<3,3>(0,0) *= 1e-6;
        P0.block<3,3>(3,3) *= 0.1;
        P0.block<3,3>(6,6) *= 0.01;
        P0.block<3,3>(9,9) *= 0.01;
        P0.block<3,3>(12,12) *= 0.001;
        P0(15,15) *= 0.001;

        EKF16d::NominalVector x0;
        x0 << p0, v0, q0.coeffs(), ba0, bg0, bb0;
        ekf.initialize(x0, P0);
    }
public:
    Ekf16Tests() : ekf(SENSE_HAT_DATA_PARAMS)
    {
    }
};

// 1. Initialization Test
TEST_F(Ekf16Tests, InitializationCorrect) {
    
    EXPECT_TRUE(ekf.pos().isApprox(p0));
    EXPECT_TRUE(ekf.vel().isApprox(v0));
    // [cite: 336] Quaternion convention check (scalar-last is Eigen default, but we check values)
    EXPECT_DOUBLE_EQ(ekf.q_vec().w(), 1.0); 
    EXPECT_DOUBLE_EQ(ekf.q_vec().x(), 0.0);
    
    // Check Covariance
    EXPECT_TRUE(ekf.getCovariance().isApprox(P0));
}

// 2. Stationary Prediction Test
// Checks if the filter maintains effectively zero velocity when proper gravity compensation is applied.
TEST_F(Ekf16Tests, StationaryHold) {
    ImuMeasurement imu;
    // Specific force in body frame when stationary = -Gravity (pointing UP)
    //  Mechanization: v_dot = C*f + g ...
    // To stay stationary, C*f should roughly equal -g
    // Assuming g ~ 9.78 at lat 0.
    imu.acc << 0, 0, -9.780327; 
    imu.gyro << 0, 0, 0;

    double dt = 0.01;
    for(int i = 0; i < 100; ++i) {
        ekf.predict(imu, dt);
    }
    
    // Velocity should remain very close to 0
    EXPECT_NEAR(ekf.vel().norm(), 0.0, 0.2); 
    
    // [cite: 327] Quaternion must remain normalized
    EXPECT_NEAR(ekf.q_vec().norm(), 1.0, 1e-8);
}

// 3. Integration Test (Constant Acceleration)
// Apply 1 m/s^2 forward acceleration (plus gravity)
TEST_F(Ekf16Tests, ForwardAcceleration) {
    ImuMeasurement imu;
    double forward_accel = 1.0;
    double g_local = 9.780327;
    
    // Accelerometer reads (Forward Accel - Gravity Vector)
    // In NED, Gravity is +Z (down). Body aligned with Nav.
    // Gravity vector in Body = [0, 0, g].
    // Specific Force f = a - g = [1, 0, 0] - [0, 0, g] = [1, 0, -g]
    imu.acc << forward_accel, 0, -g_local;
    imu.gyro << 0, 0, 0;

    double dt = 0.1;
    int steps = 10; // 1 second total
    for(int i = 0; i < steps; ++i) {
        ekf.predict(imu, dt);
    }

    // v = a * t = 1.0 * 1.0 = 1.0 m/s
    EXPECT_NEAR(ekf.vel().x(), 1.0, 0.1); 
    
    // p = 0.5 * a * t^2 = 0.5 * 1.0 * 1.0 = 0.5 m
    // Note: p.x is Latitude in radians in this implementation (from PDF def), 
    // but the `predict` function integrates meters.
    // We need to check the conversion. 
    // x_.p.x += v_x / (RM + h) * dt.
    // So p.x won't be 0.5, it will be tiny (radians).
    
    // Let's verify velocity mostly, as position depends on Earth radii calc.
    EXPECT_GT(ekf.vel().x(), 0.9);
    EXPECT_LT(ekf.vel().x(), 1.1);
}

// 4. Covariance Growth Test
// [cite: 300] P_new = Phi * P * Phi' + Qd
// Process noise should increase uncertainty during prediction without corrections.
TEST_F(Ekf16Tests, CovarianceGrowth) {
    Eigen::MatrixXd P_initial = ekf.getCovariance();
    
    ImuMeasurement imu;
    imu.acc << 0, 0, -9.8;
    imu.gyro << 0, 0, 0;
    
    // Predict for a significant time
    for(int i=0; i<50; ++i) ekf.predict(imu, 0.1);
    
    Eigen::MatrixXd P_final = ekf.getCovariance();
    
    // Trace should increase (total variance increases)
    EXPECT_GT(P_final.trace(), P_initial.trace());
}

// 5. Update Test (Position)
// [cite: 244] Innovation z = r_gnss - r_ins
// [cite: 317] State correction r+ = r- + dr
TEST_F(Ekf16Tests, GnssPositionCorrection) {
    // 1. Move state away from 0 slightly via prediction or manual injection
    // (Here we just assume it's at 0,0,0)
    
    // 2. GNSS says we are actually at 10m height (z = 10)
    Eigen::Vector3d gnss_pos(0, 0, 10.0);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.1; // High confidence
    
    double initial_z_cov = ekf.getCovariance()(2,2);
    
    ekf.update_gnss_position(gnss_pos, R);
    
    Eigen::MatrixXd P_updated = ekf.getCovariance();

    // State should move towards measurement
    // INS was 0, GNSS is 10. New state should be > 0.
    EXPECT_GT(ekf.pos().z(), 0.0);
    EXPECT_LT(ekf.pos().z(), 10.1); 
    
    // Covariance should shrink [cite: 329]
    EXPECT_LT(P_updated(2,2), initial_z_cov);
}

// 6. Quaternion Normalization Check
// [cite: 327] "Normalize quaternion: q+ <- q+ / ||q+||"
TEST_F(Ekf16Tests, QuaternionNormalizationAfterUpdate) {
    // Inject a large update that might de-normalize if not handled
    Eigen::Vector3d gnss_vel(100, 0, 0); 
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.1;
    
    ekf.update_gnss_velocity(gnss_vel, R);
    
    EXPECT_NEAR(ekf.q_vec().norm(), 1.0, 1e-15);
}

// 7. Van Loan Discretization Logic Check
// Indirectly testing [cite: 205] by checking if P stays symmetric
TEST_F(Ekf16Tests, CovarianceSymmetry) {
    ImuMeasurement imu;
    imu.acc << 0, 0, -9.8;
    imu.gyro << 0.1, 0, 0; // Some rotation
    
    ekf.predict(imu, 0.1);
    
    Eigen::MatrixXd P = ekf.getCovariance();
    
    //  Q_d symmetrized
    // [cite: 331] P updated symmetrized
    EXPECT_TRUE(P.isApprox(P.transpose(), 1e-8));
}
