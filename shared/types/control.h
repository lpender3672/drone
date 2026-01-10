#ifndef SHARED_TYPES_CONTROL_H
#define SHARED_TYPES_CONTROL_H

#include <Eigen/Dense>

namespace shared {

/**
 * Templated control effort output.
 * N is the number of control channels (e.g., 4 for quadcopter motors).
 * Values are normalized [0, 1] representing effort fraction.
 */
template<int N>
struct ControlEffort {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using VecN = Eigen::Matrix<double, N, 1>;
    
    VecN efforts = VecN::Zero();
    
    ControlEffort() = default;
    
    explicit ControlEffort(const VecN& e) : efforts(e) {}
    
    // Element access
    double operator()(int i) const { return efforts(i); }
    double& operator()(int i) { return efforts(i); }
    
    // Number of channels
    static constexpr int num_channels() { return N; }
    
    // Clamp all efforts to [0, 1]
    void clamp() {
        efforts = efforts.cwiseMax(0.0).cwiseMin(1.0);
    }
    
    // Set all to same value
    void set_all(double value) {
        efforts.setConstant(value);
    }
};


} // namespace shared

#endif // SHARED_TYPES_CONTROL_H
