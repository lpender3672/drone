#pragma once

#include "../core/interblock_data.hpp"
#include "../../shared/sensors/sensor_readings.h"

namespace sim {

class ImuData : public InterBlockData<sensors::ImuMeasurement::DataSize> {
public:
    ImuData() = default;
    
    explicit ImuData(const sensors::ImuMeasurement& meas) {
        meas.to_array(data_.data());
    }

    sensors::ImuMeasurement to_measurement() const {
        sensors::ImuMeasurement meas;
        meas.from_array(data_.data());
        return meas;
    }

    void set(const sensors::ImuMeasurement& meas) {
        meas.to_array(data_.data());
    }

    // Direct accessors for convenience
    Vec3 acc() const { return Vec3(data_[0], data_[1], data_[2]); }
    Vec3 gyro() const { return Vec3(data_[3], data_[4], data_[5]); }
    bool valid() const { return data_[7] > 0.5; }

    std::string type_name() const override { return "ImuData"; }
};

class MagData : public InterBlockData<sensors::MagMeasurement::DataSize> {
public:
    MagData() = default;
    
    explicit MagData(const sensors::MagMeasurement& meas) {
        meas.to_array(data_.data());
    }

    sensors::MagMeasurement to_measurement() const {
        sensors::MagMeasurement meas;
        meas.from_array(data_.data());
        return meas;
    }

    void set(const sensors::MagMeasurement& meas) {
        meas.to_array(data_.data());
    }

    Vec3 field() const { return Vec3(data_[0], data_[1], data_[2]); }
    bool valid() const { return data_[4] > 0.5; }

    std::string type_name() const override { return "MagData"; }
};

class BaroData : public InterBlockData<sensors::BaroMeasurement::DataSize> {
public:
    BaroData() = default;
    
    explicit BaroData(const sensors::BaroMeasurement& meas) {
        meas.to_array(data_.data());
    }

    sensors::BaroMeasurement to_measurement() const {
        sensors::BaroMeasurement meas;
        meas.from_array(data_.data());
        return meas;
    }

    void set(const sensors::BaroMeasurement& meas) {
        meas.to_array(data_.data());
    }

    double pressure_pa() const { return data_[0]; }
    double temperature_c() const { return data_[1]; }
    double altitude_m() const { return data_[2]; }
    bool valid() const { return data_[4] > 0.5; }

    std::string type_name() const override { return "BaroData"; }
};

class GnssData : public InterBlockData<sensors::GnssMeasurement::DataSize> {
public:
    GnssData() = default;
    
    explicit GnssData(const sensors::GnssMeasurement& meas) {
        meas.to_array(data_.data());
    }

    sensors::GnssMeasurement to_measurement() const {
        sensors::GnssMeasurement meas;
        meas.from_array(data_.data());
        return meas;
    }

    void set(const sensors::GnssMeasurement& meas) {
        meas.to_array(data_.data());
    }

    double latitude_deg() const { return data_[0]; }
    double longitude_deg() const { return data_[1]; }
    double altitude_m() const { return data_[2]; }
    Vec3 velocity_ned() const { return Vec3(data_[3], data_[4], data_[5]); }
    bool valid() const { return data_[11] > 0.5; }

    std::string type_name() const override { return "GnssData"; }
};

} // namespace sim