#ifndef INTERBLOCK_DATA_HPP
#define INTERBLOCK_DATA_HPP

#include <Eigen/Dense>
#include <array>
#include <string>

namespace sim {

class IInterBlockData {
public:
    IInterBlockData() : timestamp_us_(0.0) {}
    explicit IInterBlockData(uint64_t timestamp_us) : timestamp_us_(timestamp_us) {}
    virtual ~IInterBlockData() = default;
    virtual std::string type_name() const = 0;

    // Timestamp
    uint64_t timestamp() const { return timestamp_us_; }
    void set_timestamp(uint64_t t) { timestamp_us_ = t; }

protected:
    uint64_t timestamp_us_;
};

/**
 * Base class for all data exchanged between blocks.
 * Templated on signal length N for Eigen mapping.
 */
template<int N>
class InterBlockData : public IInterBlockData {
public:
    using Vector = Eigen::Matrix<double, N, 1>;
    using Map = Eigen::Map<Vector>;
    using ConstMap = Eigen::Map<const Vector>;

    static constexpr int Size = N;

    InterBlockData() : IInterBlockData(0.0) { data_.fill(0.0); }
    explicit InterBlockData(double timestamp_s) : IInterBlockData(timestamp_s) { data_.fill(0.0); }

    // Raw data access
    double* data() { return data_.data(); }
    const double* data() const { return data_.data(); }

    // Eigen vector mapping
    Map vector() { return Map(data_.data()); }
    ConstMap vector() const { return ConstMap(data_.data()); }

    // Element access
    double& operator[](int i) { return data_[i]; }
    double operator[](int i) const { return data_[i]; }

    // Type identification
    virtual std::string type_name() const { return "InterBlockData<" + std::to_string(N) + ">"; }

    static constexpr int size() { return N; }

protected:
    std::array<double, N> data_;
};

} // namespace sim

#endif // INTERBLOCK_DATA_HPP
