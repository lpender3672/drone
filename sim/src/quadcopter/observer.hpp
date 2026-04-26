#pragma once

#include <cmath>
#include <memory>

#include "../core/block.hpp"
#include "../data/sensor_reading.hpp"
#include "state.hpp"
#include "../../../shared/observer.h"

namespace sim {
namespace quadcopter {

// Sim-side scheduling wrapper around an abstract nav observer. Pick the concrete
// observer at construction time (EKF16d, future EKF16f-adapter, AHRS-7, etc.);
// this block only knows the INavObserver interface.
class QuadrotorEkfBlock : public Block {
public:
    QuadrotorEkfBlock(const std::string& name,
                      std::unique_ptr<shared::INavObserver> observer,
                      uint32_t update_period_us = 1000)
        : Block(name, update_period_us)
        , observer_(std::move(observer))
        , imu_input_("imu")
        , gnss_input_("gnss")
        , baro_input_("baro")
        , mag_input_("mag")
        , output_("state")
    {}

    InputPort<ImuData>&          imu_input()  { return imu_input_; }
    InputPort<GnssData>&         gnss_input() { return gnss_input_; }
    InputPort<BaroData>&         baro_input() { return baro_input_; }
    InputPort<MagData>&          mag_input()  { return mag_input_; }
    OutputPort<NavigationState>& output()     { return output_; }

    // Must match the origin set on GpsSensor
    void set_gnss_origin(double lat_deg, double lon_deg, double alt_m) {
        origin_lat_deg_ = lat_deg;
        origin_lon_deg_ = lon_deg;
        origin_alt_m_   = alt_m;
    }

    void initialize(const shared::TrueState& state) {
        // Convert sim's local NED state to geodetic NavigationState that the
        // observer's reset() expects. The observer is responsible for its own
        // P0 covariance policy.
        constexpr double M_PER_DEG_LAT = 111319.9;
        const double lat_rad = (origin_lat_deg_ + state.position.x() / M_PER_DEG_LAT) * M_PI / 180.0;
        const double m_per_deg_lon = M_PER_DEG_LAT * std::cos(origin_lat_deg_ * M_PI / 180.0);
        const double lon_rad = (origin_lon_deg_ + state.position.y() / m_per_deg_lon) * M_PI / 180.0;
        const double alt_m   = origin_alt_m_ - state.position.z();  // NED z-down → altitude up

        shared::NavigationState nav;
        nav.position = Vec3(lat_rad, lon_rad, alt_m);
        nav.velocity = state.velocity;
        nav.attitude = state.attitude;
        nav.angular_velocity = state.angular_velocity;
        // biases default to zero
        observer_->reset(nav);
    }

    bool update(uint64_t current_time_us) override {
        if (!is_due(current_time_us)) return false;
        mark_updated(current_time_us);

        sensors::ImuMeasurement imu = static_cast<const sensors::ImuMeasurement&>(imu_input_.get());
        imu.valid = true;

        if (imu.acc.squaredNorm() > 1.0) {
            observer_->feed_imu(imu);
        }

        // Feed GNSS / baro / mag only when the reading's timestamp advances.
        if (gnss_input_.connected && gnss_updates_enabled_) {
            uint64_t gnss_ts = gnss_input_.get().timestamp();
            if (gnss_ts != last_gnss_ts_ && gnss_ts > 0) {
                observer_->feed_gnss(static_cast<const sensors::GnssMeasurement&>(gnss_input_.value));
                last_gnss_ts_ = gnss_ts;
            }
        }

        if (baro_input_.connected && baro_updates_enabled_) {
            uint64_t baro_ts = baro_input_.get().timestamp();
            if (baro_ts != last_baro_ts_ && baro_ts > 0) {
                observer_->feed_baro(static_cast<const sensors::BaroMeasurement&>(baro_input_.value));
                last_baro_ts_ = baro_ts;
            }
        }

        if (mag_input_.connected && mag_updates_enabled_) {
            uint64_t mag_ts = mag_input_.get().timestamp();
            if (mag_ts != last_mag_ts_ && mag_ts > 0) {
                observer_->feed_mag(static_cast<const sensors::MagMeasurement&>(mag_input_.value));
                last_mag_ts_ = mag_ts;
            }
        }

        shared::NavigationState out = observer_->output();
        out.valid = true;
        static_cast<shared::NavigationState&>(output_.value) = out;
        return true;
    }

    shared::INavObserver& observer() { return *observer_; }
    void set_gnss_enabled(bool v) { gnss_updates_enabled_ = v; }
    void set_baro_enabled(bool v) { baro_updates_enabled_ = v; }
    void set_mag_enabled(bool v)  { mag_updates_enabled_  = v; }

    double origin_lat_deg_ = 52.2053;
    double origin_lon_deg_ =  0.1218;
    double origin_alt_m_   = 10.0;

private:
    std::unique_ptr<shared::INavObserver> observer_;
    InputPort<ImuData>  imu_input_;
    InputPort<GnssData> gnss_input_;
    InputPort<BaroData> baro_input_;
    InputPort<MagData>  mag_input_;
    OutputPort<NavigationState> output_;
    uint64_t last_gnss_ts_         = 0;
    uint64_t last_baro_ts_         = 0;
    uint64_t last_mag_ts_          = 0;
    bool     gnss_updates_enabled_ = true;
    bool     baro_updates_enabled_ = true;
    bool     mag_updates_enabled_  = true;
};

} // namespace quadcopter
} // namespace sim
