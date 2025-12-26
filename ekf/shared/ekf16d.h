#ifndef EKF16D_H
#define EKF16D_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <iostream>

#include "ekf.h"

const int DIM_NOMINAL = 17;
const int DIM_ERROR = 16;
const int DIM_NOISE = 13;

enum ErrIdx {
    ERR_POS   = 0,
    ERR_VEL   = 3,
    ERR_ATT   = 6,
    ERR_BA    = 9,
    ERR_BG    = 12,
    ERR_BBARO = 15
};

enum NomIdx {
    NOM_POS   = 0,
    NOM_VEL   = 3,
    NOM_QUAT  = 6,   // 4 elements
    NOM_BA    = 10,
    NOM_BG    = 13,
    NOM_BBARO = 16
};

class EKF16d : public EKF<DIM_NOMINAL, DIM_ERROR, DIM_NOISE> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EKF16d(const EkfErrorParameters& p);

    // Prediction Step [Source: 282]
    void predict(const ImuMeasurement& imu, double dt) override;

    // Measurement Updates [Source: 236]
    void update_gnss_position(const Eigen::Vector3d& pos_gnss, const Eigen::Matrix3d& R) override;
    void update_gnss_velocity(const Eigen::Vector3d& vel_gnss, const Eigen::Matrix3d& R) override;
    void update_barometer(double altitude, double R_var) override;
    void update_magnetometer(const Eigen::Vector3d& mag_body, const Eigen::Matrix3d& R) override;

    void (*debugCallback)(const char* label) = nullptr;

    // Nominal state accessors
    Eigen::Vector3d pos() const { return x_.segment<3>(NOM_POS); }
    Eigen::Vector3d vel() const { return x_.segment<3>(NOM_VEL); }
    Eigen::Vector3d ba()  const { return x_.segment<3>(NOM_BA);  }
    Eigen::Vector3d bg()  const { return x_.segment<3>(NOM_BG);  }
    double bbaro() const { return x_(NOM_BBARO); }

    // Mutable references
    Eigen::VectorBlock<NominalVector,3> pos() { return x_.segment<3>(NOM_POS); }
    Eigen::VectorBlock<NominalVector,3> vel() { return x_.segment<3>(NOM_VEL); }
    Eigen::VectorBlock<NominalVector,3> ba()  { return x_.segment<3>(NOM_BA);  }
    Eigen::VectorBlock<NominalVector,3> bg()  { return x_.segment<3>(NOM_BG);  }
    double& bbaro() { return x_(NOM_BBARO); }

    // Quaternion stored as [w x y z]
    Eigen::VectorBlock<const NominalVector,4> q_vec() const {
        return x_.segment<4>(NOM_QUAT);
    }
    Eigen::VectorBlock<NominalVector,4> q_vec() {
        return x_.segment<4>(NOM_QUAT);
    }

private:

    // Constants [Source: 148-150, 140]
    const double R0 = 6378137.0;            // Equatorial Radius
    const double e2 = 0.00669437999014;     // Eccentricity squared
    const double g0 = 9.80665;              // Gravity
    const double WE = 7.292115e-5;          // Earth rotation rate

    Eigen::Vector3d tau_a_;           // Accel bias correlation time [s]
    Eigen::Vector3d tau_g_;           // Gyro bias correlation time [s]
    double tau_bbaro_;               // Baro bias correlation time [s]

    // Helpers
    void compute_radius(double lat, double& RM, double& RN);
    void inject_error(const ErrorVector& dx);

    inline Eigen::Quaterniond quat_from_state(const Eigen::Vector4d& qv) {
        return Eigen::Quaterniond(qv(0), qv(1), qv(2), qv(3));
    }

    inline Eigen::Vector4d state_from_quat(const Eigen::Quaterniond& q) {
        return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
    }
};

#endif // ES_EKF_H