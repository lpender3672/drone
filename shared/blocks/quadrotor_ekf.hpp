#pragma once

#include <cmath>
#include <memory>

#include "../core/block.hpp"
#include "../data/gnss_origin.hpp"
#include "../observer.h"
#include "../sensors/sensor_readings.h"
#include "../types/state.h"

namespace shared {

// Scheduling wrapper around an abstract nav observer. Pick the concrete
// observer at construction (EKF16d, future EKF16f, AHRS-7); this block
// only knows the INavObserver interface.
//
// Templated on the four sensor data types and the navigation-state output
// type so the same class works for sim (where data types are wrapper
// subclasses adding sim's InterBlockData<> for logging) and for embedded
// (where the bare sensors::*Measurement types are used directly).
template<
    typename TImu  = sensors::ImuMeasurement,
    typename TGnss = sensors::GnssMeasurement,
    typename TBaro = sensors::BaroMeasurement,
    typename TMag  = sensors::MagMeasurement,
    typename TNavOut = NavigationState
>
class QuadrotorEkfBlockT : public Block {
public:
    QuadrotorEkfBlockT(const std::string& name,
                       std::unique_ptr<INavObserver> observer,
                       uint32_t update_period_us = 1000)
        : Block(name, update_period_us)
        , observer_(std::move(observer))
        , imu_input_("imu")
        , gnss_input_("gnss")
        , baro_input_("baro")
        , mag_input_("mag")
        , output_("state")
    {
        // Direct Block subclass — register the ports manually so graph-spec
        // wiring (Tier 2) can find them by name.
        this->register_port(imu_input_);
        this->register_port(gnss_input_);
        this->register_port(baro_input_);
        this->register_port(mag_input_);
        this->register_port(output_);
    }

    InputPort<TImu>&          imu_input()  { return imu_input_; }
    InputPort<TGnss>&         gnss_input() { return gnss_input_; }
    InputPort<TBaro>&         baro_input() { return baro_input_; }
    InputPort<TMag>&          mag_input()  { return mag_input_; }
    OutputPort<TNavOut>&      output()     { return output_; }

    // The system pushes the same origin to GpsSensor, BaroSensor, and this
    // block — they have to agree or the EKF's geodetic measurements will be
    // inconsistent with the truth that the sensors synthesise.
    void set_origin(const GnssOrigin& origin) { origin_ = origin; }
    const GnssOrigin& origin() const { return origin_; }

    void initialize(const TrueState& state) {
        // Convert local NED state to a geodetic NavigationState that the
        // observer's reset() expects. The observer is responsible for its
        // own P0 covariance policy.
        constexpr double M_PER_DEG_LAT = 111319.9;
        const double lat_rad = (origin_.lat_deg + state.position.x() / M_PER_DEG_LAT) * M_PI / 180.0;
        const double m_per_deg_lon = M_PER_DEG_LAT * std::cos(origin_.lat_deg * M_PI / 180.0);
        const double lon_rad = (origin_.lon_deg + state.position.y() / m_per_deg_lon) * M_PI / 180.0;
        const double alt_m   = origin_.alt_m - state.position.z();  // NED z-down → altitude up

        NavigationState nav;
        nav.position = Vec3(lat_rad, lon_rad, alt_m);
        nav.velocity = state.velocity;
        nav.attitude = state.attitude;
        nav.angular_velocity = state.angular_velocity;
        observer_->reset(nav);
    }

    bool update(uint64_t current_time_us) override {
        if (!is_due(current_time_us)) return false;
        mark_updated(current_time_us);

        // Gate IMU feeds by the reading's timestamp so the same sample
        // doesn't get integrated twice if the EKF ticks faster than the
        // IMU (e.g. embedded EKF at 100 Hz with IMU at 100 Hz can drift
        // to 99 Hz in either direction; sim sensors run at EKF rate so
        // this is a no-op there).
        const auto& imu = static_cast<const sensors::ImuMeasurement&>(imu_input_.get());
        if (imu.timestamp_us != last_imu_ts_ && imu.acc.squaredNorm() > 1.0) {
            observer_->feed_imu(imu);
            last_imu_ts_ = imu.timestamp_us;
        }

        // Feed GNSS / baro / mag only when the reading's timestamp advances.
        if (gnss_input_.connected && gnss_updates_enabled_) {
            const auto& gnss = static_cast<const sensors::GnssMeasurement&>(gnss_input_.get());
            if (gnss.timestamp_us != last_gnss_ts_ && gnss.timestamp_us > 0) {
                observer_->feed_gnss(gnss);
                last_gnss_ts_ = gnss.timestamp_us;
            }
        }

        if (baro_input_.connected && baro_updates_enabled_) {
            const auto& baro = static_cast<const sensors::BaroMeasurement&>(baro_input_.get());
            if (baro.timestamp_us != last_baro_ts_ && baro.timestamp_us > 0) {
                observer_->feed_baro(baro);
                last_baro_ts_ = baro.timestamp_us;
            }
        }

        if (mag_input_.connected && mag_updates_enabled_) {
            const auto& mag = static_cast<const sensors::MagMeasurement&>(mag_input_.get());
            if (mag.timestamp_us != last_mag_ts_ && mag.timestamp_us > 0) {
                observer_->feed_mag(mag);
                last_mag_ts_ = mag.timestamp_us;
            }
        }

        NavigationState out = observer_->output();
        out.valid = true;
        output_.value.assign_nav(out);
        return true;
    }

    INavObserver& observer() { return *observer_; }
    void set_gnss_enabled(bool v) { gnss_updates_enabled_ = v; }
    void set_baro_enabled(bool v) { baro_updates_enabled_ = v; }
    void set_mag_enabled(bool v)  { mag_updates_enabled_  = v; }

private:
    GnssOrigin origin_;
    std::unique_ptr<INavObserver> observer_;
    InputPort<TImu>     imu_input_;
    InputPort<TGnss>    gnss_input_;
    InputPort<TBaro>    baro_input_;
    InputPort<TMag>     mag_input_;
    OutputPort<TNavOut> output_;
    uint64_t last_imu_ts_          = 0;
    uint64_t last_gnss_ts_         = 0;
    uint64_t last_baro_ts_         = 0;
    uint64_t last_mag_ts_          = 0;
    bool     gnss_updates_enabled_ = true;
    bool     baro_updates_enabled_ = true;
    bool     mag_updates_enabled_  = true;
};

// Default instantiation for embedded targets — bare sensors::*Measurement
// types and the shared NavigationState.
using QuadrotorEkfBlock = QuadrotorEkfBlockT<>;

} // namespace shared
