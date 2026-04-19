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
        shared::ObservedState obs(state);
        ekf_.reset(obs);
    }

    bool update(uint64_t current_time_us) override {
        if (!is_due(current_time_us)) return false;
        mark_updated(current_time_us);

        // Feed IMU using explicit dt from block timing rather than sensor timestamps
        // (sensor timestamps are unreliable due to latency buffering)
        sensors::ImuMeasurement imu = static_cast<const sensors::ImuMeasurement&>(imu_input_.value);
        imu.valid = true;

        if (imu.acc.squaredNorm() > 1.0) {
            double dt = get_dt_us(current_time_us) * 1e-6;
            ekf_.predict(imu, dt);
        }

        // Feed GNSS only when a new reading arrives (timestamp changed)
        if (gnss_input_.connected) {
            uint64_t gnss_ts = gnss_input_.value.timestamp();
            if (gnss_ts != last_gnss_ts_ && gnss_ts > 0) {
                const auto& g = gnss_input_.value;
                gnss_to_ned_update(g);
                last_gnss_ts_ = gnss_ts;
            }
        }

        shared::ObservedState ekf_out = ekf_.output();
        ekf_out.valid = true;
        static_cast<shared::ObservedState&>(output_.value) = ekf_out;

        return true;
    }

    EKF16d& ekf() { return ekf_; }

private:
    void gnss_to_ned_update(const GnssData& g) {
        constexpr double M_PER_DEG_LAT = 111319.9;
        double m_per_deg_lon = M_PER_DEG_LAT * std::cos(origin_lat_deg_ * M_PI / 180.0);

        Eigen::Vector3d pos_ned;
        pos_ned.x() = (g.latitude_deg  - origin_lat_deg_) * M_PER_DEG_LAT;
        pos_ned.y() = (g.longitude_deg - origin_lon_deg_) * m_per_deg_lon;
        pos_ned.z() = -(g.altitude_m   - origin_alt_m_);

        double hdop = g.hdop > 0.0f ? g.hdop : 1.0f;
        double vdop = g.vdop > 0.0f ? g.vdop : 1.5f;
        Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * (hdop * hdop * 2.5);
        R_pos(2, 2) = vdop * vdop * 4.0;

        ekf_.update_gnss_position(pos_ned, R_pos);

        constexpr double vel_var = 0.1;
        ekf_.update_gnss_velocity(g.velocity_ned,
                                  Eigen::Matrix3d::Identity() * vel_var);
    }

    EKF16d              ekf_;
    InputPort<ImuData>  imu_input_;
    InputPort<GnssData> gnss_input_;
    OutputPort<ObservedState> output_;
    uint64_t last_gnss_ts_   = 0;
    double   origin_lat_deg_ = 52.2053;
    double   origin_lon_deg_ =  0.1218;
    double   origin_alt_m_   = 10.0;
};

} // namespace quadcopter
} // namespace sim
