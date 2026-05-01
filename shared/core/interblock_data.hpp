#pragma once

#include <Eigen/Dense>
#include <array>

namespace shared {

// Fixed-size array container with an Eigen view for inter-block signal
// data. The Tier 2 graph runtime tags ports by type at construction time
// (see shared::detail::type_tag in core/block.hpp), so this class only
// owns storage and basic accessors — no virtual interface, no per-instance
// timestamp, no logger hooks. Concrete signal types (Scalar, MotorEfforts,
// PidInput, etc. in shared/data/state.hpp) inherit it for the data_ array
// plus their own typed getters/setters.
template<int N>
class InterBlockData {
public:
    using Vector   = Eigen::Matrix<double, N, 1>;
    using Map      = Eigen::Map<Vector>;
    using ConstMap = Eigen::Map<const Vector>;

    static constexpr int Size = N;

    InterBlockData() { data_.fill(0.0); }

    double*       data()       { return data_.data(); }
    const double* data() const { return data_.data(); }

    Map      vector()       { return Map(data_.data()); }
    ConstMap vector() const { return ConstMap(data_.data()); }

    double& operator[](int i)       { return data_[i]; }
    double  operator[](int i) const { return data_[i]; }

    static constexpr int size() { return N; }

protected:
    std::array<double, N> data_;
};

} // namespace shared
