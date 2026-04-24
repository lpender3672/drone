#ifndef EKF16_H
#define EKF16_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

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

// Scalar-templated 16-error-state INS EKF (core math only; no IObserver).
// Instantiated as EKF16<double> and EKF16<float> in ekf16.cpp.
template<typename Scalar>
class EKF16 : public EKF<Scalar, DIM_NOMINAL, DIM_ERROR, DIM_NOISE> {
public:
    using Base = EKF<Scalar, DIM_NOMINAL, DIM_ERROR, DIM_NOISE>;
    using typename Base::Vec3;
    using typename Base::Vec4;
    using typename Base::Mat3;
    using typename Base::Quat;
    using typename Base::NominalVector;
    using typename Base::CovMatrix;
    using typename Base::ErrorVector;

    // Pull protected base members into scope so derived method bodies don't need `this->`.
    using Base::x_;
    using Base::P_;
    using Base::Qc_;
    using Base::reserver_;
    using Base::skew;
    using Base::initialize;
    template<int M> using MeasCov = typename Base::template MeasCov<M>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EKF16(const EkfErrorParameters& p);

    // Core math
    void predict(const ImuMeasurement& imu, double dt);
    void update_gnss_position(const Vec3& pos_gnss, const Mat3& R);
    void update_gnss_velocity(const Vec3& vel_gnss, const Mat3& R);
    void update_barometer(Scalar altitude, Scalar R_var);
    void update_magnetometer(const Vec3& mag_body, const Mat3& R);
    void update_gravity(const Vec3& f_body);

    void (*debugCallback)(const char* label) = nullptr;

    // Nominal-state accessors
    Vec3 pos() const { return x_.template segment<3>(NOM_POS); }
    Vec3 vel() const { return x_.template segment<3>(NOM_VEL); }
    Vec3 ba()  const { return x_.template segment<3>(NOM_BA);  }
    Vec3 bg()  const { return x_.template segment<3>(NOM_BG);  }
    Scalar bbaro() const { return x_(NOM_BBARO); }

    Eigen::VectorBlock<NominalVector, 3> pos() { return x_.template segment<3>(NOM_POS); }
    Eigen::VectorBlock<NominalVector, 3> vel() { return x_.template segment<3>(NOM_VEL); }
    Eigen::VectorBlock<NominalVector, 3> ba()  { return x_.template segment<3>(NOM_BA);  }
    Eigen::VectorBlock<NominalVector, 3> bg()  { return x_.template segment<3>(NOM_BG);  }
    Scalar& bbaro() { return x_(NOM_BBARO); }

    Eigen::VectorBlock<const NominalVector, 4> q_vec() const { return x_.template segment<4>(NOM_QUAT); }
    Eigen::VectorBlock<NominalVector, 4> q_vec() { return x_.template segment<4>(NOM_QUAT); }

protected:
    static constexpr Scalar WE_ = Scalar(7.292115e-5);  // Earth rotation rate [rad/s]

    Vec3 tau_a_;           // Accel bias correlation time [s]
    Vec3 tau_g_;           // Gyro bias correlation time [s]
    Scalar tau_bbaro_;     // Baro bias correlation time [s]

    // Cached IMU-derived data
    Vec3 last_omega_{Vec3::Zero()};
    double last_imu_dt_ = 0.01;
    uint64_t last_imu_timestamp_us_ = 0;
    uint64_t last_gravity_update_us_ = 0;

    // Measurement noise (set from params)
    Scalar baro_noise_var_ = Scalar(1.0);
    Vec3 gravity_noise_var_{Vec3::Constant(Scalar(1e-4))};

    void inject_error(const ErrorVector& dx) override;

    static Quat quat_from_state(const Vec4& qv) {
        return Quat(qv(0), qv(1), qv(2), qv(3));
    }
    static Vec4 state_from_quat(const Quat& q) {
        return Vec4(q.w(), q.x(), q.y(), q.z());
    }
};

#endif // EKF16_H
