#pragma once

#include <string>
#include <vector>
#include <functional>
#include "interblock_data.hpp"

namespace sim {

// Forward declaration

template<typename T>
struct InputPort {
    std::string name;
    T value{};
    bool connected = false;
    
    explicit InputPort(const std::string& n) : name(n) {}
    
    void set(const T& v) { value = v; connected = true; }
    const T& get() const { return value; }
};

template<typename T>
struct OutputPort {
    std::string name;
    T value{};
    
    explicit OutputPort(const std::string& n) : name(n) {}
    
    void set(const T& v) { value = v; }
    const T& get() const { return value; }
};

class Block {
public:
    using OutputCallback = std::function<void(const std::string&, const IInterBlockData&)>;
    
    explicit Block(const std::string& name, double update_rate_hz = 0.0)
        : name_(name)
        , update_rate_hz_(update_rate_hz)
        , update_period_s_(update_rate_hz > 0.0 ? 1.0 / update_rate_hz : 0.0)
        , last_update_time_s_(-1e9)
    {}
    
    virtual ~Block() = default;

    virtual bool update(double current_time_s) = 0;

    const std::string& name() const { return name_; }
    double update_rate_hz() const { return update_rate_hz_; }
    double update_period_s() const { return update_period_s_; }
    
    bool is_due(double current_time_s) const {
        if (update_rate_hz_ <= 0.0) return true;
        return (current_time_s - last_update_time_s_) >= update_period_s_ - 1e-9;
    }

    void add_output_callback(OutputCallback cb) {
        output_callbacks_.push_back(std::move(cb));
    }
    
    void clear_callbacks() {
        output_callbacks_.clear();
    }

protected:
    void notify_output(const IInterBlockData& data) {
        for (auto& cb : output_callbacks_) {
            cb(name_, data);
        }
    }
    
    void mark_updated(double current_time_s) {
        last_update_time_s_ = current_time_s;
    }

    std::string name_;
    double update_rate_hz_;
    double update_period_s_;
    double last_update_time_s_;
    std::vector<OutputCallback> output_callbacks_;
};

template<typename TIn, typename TOut>
class TypedBlock : public Block {
public:
    using InputType = TIn;
    using OutputType = TOut;
    
    TypedBlock(const std::string& name, 
               const std::string& input_name,
               const std::string& output_name,
               double update_rate_hz = 0.0)
        : Block(name, update_rate_hz)
        , input_(input_name)
        , output_(output_name)
    {}

    InputPort<TIn>& input() { return input_; }
    const InputPort<TIn>& input() const { return input_; }
    
    OutputPort<TOut>& output() { return output_; }
    const OutputPort<TOut>& output() const { return output_; }

protected:
    InputPort<TIn> input_;
    OutputPort<TOut> output_;
};

} // namespace sim
