#pragma once

#include <cstdint>
#include <string>
#include <Eigen/Dense>

namespace sim {

// Common Eigen typedefs
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Quat = Eigen::Quaterniond;

/**
 * Base class for all data exchanged between blocks.
 * Provides timestamping and type identification.
 */
class InterBlockData {
public:
    explicit InterBlockData(double timestamp_s = 0.0) 
        : timestamp_s_(timestamp_s) {}
    
    virtual ~InterBlockData() = default;
    
    // Timestamp in seconds from simulation start
    double timestamp() const { return timestamp_s_; }
    void set_timestamp(double t) { timestamp_s_ = t; }
    
    // Type name for logging/debugging
    virtual std::string type_name() const { return "InterBlockData"; }
    
    // Clone for when we need to buffer/store data
    virtual InterBlockData* clone() const { 
        return new InterBlockData(*this); 
    }

protected:
    double timestamp_s_;
};

} // namespace sim
