#pragma once
#include "../core/interblock_data.hpp"
#include "../../shared/sensors/sensor_readings.h"

namespace sim {

class ImuData : public InterBlockData<sensors::ImuMeasurement::DataSize>, 
                public sensors::ImuMeasurement {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    ImuData() = default;
    
    explicit ImuData(const sensors::ImuMeasurement& meas) 
        : sensors::ImuMeasurement(meas) {}
    
    // Sync to internal array for interblock transmission
    void sync_to_array() { to_array(data_.data()); }
    void sync_from_array() { from_array(data_.data()); }
    
    std::string type_name() const override { return "ImuData"; }
};

class MagData : public InterBlockData<sensors::MagMeasurement::DataSize>, 
                public sensors::MagMeasurement {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MagData() = default;
    
    explicit MagData(const sensors::MagMeasurement& meas) 
        : sensors::MagMeasurement(meas) {}
    
    void sync_to_array() { to_array(data_.data()); }
    void sync_from_array() { from_array(data_.data()); }
    
    std::string type_name() const override { return "MagData"; }
};

class BaroData : public InterBlockData<sensors::BaroMeasurement::DataSize>, 
                 public sensors::BaroMeasurement {
public:
    BaroData() = default;
    
    explicit BaroData(const sensors::BaroMeasurement& meas) 
        : sensors::BaroMeasurement(meas) {}
    
    void sync_to_array() { to_array(data_.data()); }
    void sync_from_array() { from_array(data_.data()); }
    
    std::string type_name() const override { return "BaroData"; }
};

class GnssData : public InterBlockData<sensors::GnssMeasurement::DataSize>, 
                 public sensors::GnssMeasurement {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    GnssData() = default;
    
    explicit GnssData(const sensors::GnssMeasurement& meas) 
        : sensors::GnssMeasurement(meas) {}
    
    void sync_to_array() { to_array(data_.data()); }
    void sync_from_array() { from_array(data_.data()); }
    
    std::string type_name() const override { return "GnssData"; }
};

} // namespace sim