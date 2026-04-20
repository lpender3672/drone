#pragma once

#include <cmath>
#include "../core/block.hpp"
#include "../data/sensor_reading.hpp"
#include "state.hpp"
#include "ekf16d.h"

namespace sim {
namespace quadcopter {

class QuadrotorEkfBlock : public Block {
public:
    QuadrotorEkfBlock(const std::string& name,
                      const EkfErrorParameters& params,
                      uint32_t update_period_us = 1000)
        : Block(name, update_period_us)
        , ekf_(params)
        , imu_input_("imu")
        , gnss_input_("gnss")
        , output_("state")
    {}

    InputPort<ImuData>&        imu_input()  { return imu_input_; }
    InputPort<GnssData>&       gnss_input() { return gnss_input_; }
    OutputPort<ObservedState>& output()     { return output_; }

    // Must match the origin set on GpsSensor
    void set_gnss_origin(double lat_deg, double lon_deg, double alt_m) {
        origin_lat_deg_ = lat_deg;
        origin_lon_deg_ = lon_deg;
        origin_alt_m_   = alt_m;
    }

    void initialize(const shared::TrueState& state) {
        // Build nominal state vector: [pos(3), vel(3), q(4), ba(3), bg(3), bbaro(1)]
        // EKF pos is geodetic (lat_rad, lon_rad, alt_m); convert from NED metres.
        constexpr double M_PER_DEG_LAT = 111319.9;
        double lat_rad = (origin_lat_deg_ + state.position.x() / M_PER_DEG_LAT) * M_PI / 180.0;
        double m_per_deg_lon = M_PER_DEG_LAT * std::cos(origin_lat_deg_ * M_PI / 180.0);
        double lon_rad = (origin_lon_deg_ + state.position.y() / m_per_deg_lon) * M_PI / 180.0;
        double alt_m   = origin_alt_m_ - state.position.z();  // NED z-down → altitude up

        EKF16d::NominalVector x0;
        x0.setZero();
        x0(0) = lat_rad;
        x0(1) = lon_rad;
        x0(2) = alt_m;
        x0.segment<3>(3)  = state.velocity;
        x0(6) = state.attitude.w();
        x0(7) = state.attitude.x();
        x0(8) = state.attitude.y();
        x0(9) = state.attitude.z();
        // biases start at zero

        // Tight initial covariance — we know the exact starting state.
        // Large P_vel causes GNSS velocity updates to inject huge spurious
        // attitude corrections through the P_vel_att coupling.
        EKF16d::CovMatrix P0;
        P0.setZero();
        // pos variance in geodetic units: 1 m / RM ≈ 1.5e-7 rad for horizontal, 1 m² for alt
        P0(0,0) = 1.0 / (6335439.0 * 6335439.0);
        P0(1,1) = 1.0 / (6378388.0 * 6378388.0);
        P0(2,2) = 1.0;
        P0.block<3,3>(3,3)   = Eigen::Matrix3d::Identity() * 1e-4;    // vel: 0.01 m/s
        P0.block<3,3>(6,6)   = Eigen::Matrix3d::Identity() * 1e-6;    // att: 0.001 rad
        P0.block<3,3>(9,9)   = Eigen::Matrix3d::Identity() * 1e-6;    // accel bias
        P0.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * 1e-6;    // gyro bias
        P0(15,15)            = 1e-4;                                   // baro bias

        ekf_.initialize(x0, P0);
    }

    bool update(uint64_t current_time_us) override {
        if (!is_due(current_time_us)) return false;

        // Compute dt BEFORE mark_updated: get_dt_us uses last_update_time_us_
        double dt = get_dt_us(current_time_us) * 1e-6;
        mark_updated(current_time_us);

        sensors::ImuMeasurement imu = static_cast<const sensors::ImuMeasurement&>(imu_input_.value);
        imu.valid = true;

        if (imu.acc.squaredNorm() > 1.0) {
            ekf_.predict(imu, dt);
        }

        // Feed GNSS only when a new reading arrives (timestamp changed)
        if (gnss_input_.connected && gnss_updates_enabled_) {
            uint64_t gnss_ts = gnss_input_.value.timestamp();
            if (gnss_ts != last_gnss_ts_ && gnss_ts > 0) {
                ekf_.feed_gnss(static_cast<const sensors::GnssMeasurement&>(gnss_input_.value));
                last_gnss_ts_ = gnss_ts;
            }
        }

        shared::ObservedState ekf_out = ekf_.output();
        // output() uses last_omega_ which is only set via feed_imu(); use gyro directly
        if (imu.acc.squaredNorm() > 1.0) {
            ekf_out.angular_velocity = imu.gyro - ekf_.bg();
        }
        ekf_out.valid = true;
        static_cast<shared::ObservedState&>(output_.value) = ekf_out;

        return true;
    }

    EKF16d& ekf() { return ekf_; }
    void set_gnss_enabled(bool v) { gnss_updates_enabled_ = v; }

    EKF16d              ekf_;
    InputPort<ImuData>  imu_input_;
    InputPort<GnssData> gnss_input_;
    OutputPort<ObservedState> output_;
    uint64_t last_gnss_ts_       = 0;
    bool     gnss_updates_enabled_ = true;
    double   origin_lat_deg_ = 52.2053;
    double   origin_lon_deg_ =  0.1218;
    double   origin_alt_m_   = 10.0;
};

} // namespace quadcopter
} // namespace sim
