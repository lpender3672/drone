#pragma once

#include "../core/inter_block_data.hpp"

namespace sim {

/**
 * Non-template base for control efforts (enables runtime polymorphism).
 */
struct ControlEffortsBase : public InterBlockData {
    using InterBlockData::InterBlockData;
    
    virtual int num_channels() const = 0;
    virtual double effort(int i) const = 0;
    virtual double& effort(int i) = 0;
};

/**
 * Generic control efforts base class.
 * Template parameter N specifies the number of control channels.
 * Values are normalised [0, 1] representing effort fraction.
 */
template<int N>
struct ControlEfforts : public ControlEffortsBase {
    using VecN = Eigen::Matrix<double, N, 1>;
    
    VecN efforts = VecN::Zero();

    ControlEfforts() = default;
    explicit ControlEfforts(double timestamp_s) : ControlEffortsBase(timestamp_s) {}
    
    explicit ControlEfforts(const VecN& e, double timestamp_s = 0.0)
        : ControlEffortsBase(timestamp_s)
        , efforts(e)
    {}

    // Runtime channel access (from base)
    int num_channels() const override { return N; }
    double effort(int i) const override { return efforts(i); }
    double& effort(int i) override { return efforts(i); }

    // Default type_name (can be overridden)
    std::string type_name() const override { 
        return "ControlEfforts<" + std::to_string(N) + ">"; 
    }

    // Default clone (can be overridden)
    InterBlockData* clone() const override {
        return new ControlEfforts<N>(*this);
    }

    // Clamp all efforts to [0, 1]
    void clamp() {
        efforts = efforts.cwiseMax(0.0).cwiseMin(1.0);
    }

    // Convenience accessors
    double& operator[](int i) { return efforts(i); }
    double operator[](int i) const { return efforts(i); }

    // Total effort (sum of efforts, useful for debugging)
    double total() const {
        return efforts.sum();
    }

    // Number of control channels (compile-time)
    static constexpr int size() { return N; }
};

} // namespace sim
