#pragma once

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include "interblock_data.hpp"

namespace shared {

// Forward declaration needed for InputPort::source pointer
template<typename T>
struct OutputPort;

template<typename T>
struct InputPort {
    std::string name;
    bool connected = false;
    const OutputPort<T>* source = nullptr;

    explicit InputPort(const std::string& n) : name(n) {}

    // Push a value into a port that has no upstream block (e.g. an external
    // reference input, or a parent composite forwarding into a child).
    // Mutually exclusive with connect(): once a source pointer is wired,
    // set() is a no-op write that get() will ignore.
    void set(const T& v) { value_ = v; connected = true; }

    // Reads through the source pointer if wired via connect(); otherwise
    // returns the value last written by set(). Always use get() — never
    // read the underlying storage directly, or you'll see stale data once
    // the port gets wired.
    const T& get() const {
        return source ? source->value : value_;
    }

private:
    T value_{};
};

template<typename T>
struct OutputPort {
    std::string name;
    T value{};

    explicit OutputPort(const std::string& n) : name(n) {}

    void set(const T& v) { value = v; }
    const T& get() const { return value; }
};

// Wire an output port directly to an input port.
// After this call, in.get() reads through the pointer — zero-copy.
template<typename T>
void connect(OutputPort<T>& out, InputPort<T>& in) {
    in.source    = &out;
    in.connected = true;
}

class Block {
public:
    using OutputCallback =
        std::function<void(const std::string&, const IInterBlockData&)>;

    // update_period_us = 0 → always due
    explicit Block(const std::string& name,
                   uint32_t update_period_us = 0)
        : name_(name)
        , update_period_us_(update_period_us)
        , last_update_time_us_(0)
        , has_updated_(false)
    {}

    virtual ~Block() = default;

    // current_time_us must be monotonic
    virtual bool update(uint64_t current_time_us) = 0;

    uint64_t get_dt_us(uint64_t current_time_us) const {
        if (!has_updated_) {
            return update_period_us_;
        }
        return current_time_us - last_update_time_us_;
    }

    const std::string& name() const { return name_; }
    uint32_t update_period_us() const { return update_period_us_; }

    bool is_due(uint64_t current_time_us) const {
        if (update_period_us_ == 0) {
            return true;
        }
        if (!has_updated_) {
            return true;
        }
        return get_dt_us(current_time_us) >= update_period_us_;
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

    void mark_updated(uint64_t current_time_us) {
        last_update_time_us_ = current_time_us;
        has_updated_ = true;
    }

    std::string name_;
    uint32_t update_period_us_;
    uint64_t last_update_time_us_;
    bool has_updated_;

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
               uint32_t update_period_us = 0)
        : Block(name, update_period_us)
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

template<typename TOut>
class SourceBlock : public Block {
public:
    SourceBlock(const std::string& name,
                const std::string& output_name,
                uint32_t update_period_us = 0)
        : Block(name, update_period_us)
        , output_(output_name)
    {}

    OutputPort<TOut>& output() { return output_; }
    const OutputPort<TOut>& output() const { return output_; }

protected:
    OutputPort<TOut> output_;
};

// A block that owns and schedules a set of child blocks.
// Calling update() on a CompositeBlock ticks each child that is due.
// Children are updated in the order they were added via add_child().
class CompositeBlock : public Block {
public:
    explicit CompositeBlock(const std::string& name, uint32_t update_period_us = 0)
        : Block(name, update_period_us) {}

    bool update(uint64_t t) override {
        for (auto& child : children_)
            if (child->is_due(t)) child->update(t);
        mark_updated(t);
        return true;
    }

protected:
    template<typename T>
    T* add_child(std::unique_ptr<T> block) {
        T* ptr = block.get();
        children_.push_back(std::move(block));
        return ptr;
    }

private:
    std::vector<std::unique_ptr<Block>> children_;
};

} // namespace shared
