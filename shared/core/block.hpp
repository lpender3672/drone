#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <memory>
#include "interblock_data.hpp"
#include "misuse.hpp"

namespace shared {

namespace detail {

// Compile-time per-type tag. Each instantiation of `type_tag<T>` has its
// own static char `marker`, and `&marker` gives a unique address per T —
// usable as a runtime "type id" via pointer equality. Avoids RTTI so we
// can build under the embedded toolchain's default `-fno-rtti`.
template<typename T>
struct type_tag {
    static const void* id() {
        static const char marker = 0;
        return &marker;
    }
};

} // namespace detail

// Type-erased port base. Lets graph-driven wiring (Tier 2) look up ports
// by string and connect them without knowing T at the wiring site. The
// typed templates below — InputPort<T>, OutputPort<T> — derive from
// this and override the virtuals.
class IPort {
public:
    virtual ~IPort() = default;

    virtual std::string_view port_name() const = 0;
    // Type identity for runtime port matching. Compare via pointer
    // equality: two ports share a type iff their type_id() values are
    // equal. (Implementations return &detail::type_tag<T>::marker.)
    virtual const void*      type_id()   const = 0;
    virtual bool             is_input()  const = 0;

    // Type-erased read of the port's current value. Returns a `const T*`
    // to the live value for the port's T (cast back via type_id() check).
    // For an output, this is the port's `value`. For an input, it follows
    // the same source/value_ logic as `get()` — so reading a wired input
    // sees through to the upstream output, and an unwired one sees the
    // last `set()` value.
    //
    // Used by the signal-trace logger (shared/core/tracer.hpp) so a
    // pre-registered (T → serializer) table can dump arbitrary ports
    // to CSV without knowing T at the trace site.
    virtual const void* read_erased() const = 0;

    // Called on an OUTPUT port to wire it to an input. The default impl
    // throws — only outputs can drive a connection. OutputPort<T>
    // overrides to type-check `in_port` against itself and call the
    // existing typed connect() free function.
    virtual void connect_to(IPort& in_port) {
        (void)in_port;
        detail::invalid_argument(
            "IPort::connect_to: only OutputPort can be a connection source");
    }
};

// Forward declaration needed for InputPort::source pointer
template<typename T>
struct OutputPort;

template<typename T>
struct InputPort;

// Forward-declare the free connect() so OutputPort::connect_to() (defined
// before connect()'s definition below) can call it.
template<typename T>
void connect(OutputPort<T>& out, InputPort<T>& in);

template<typename T>
struct InputPort : public IPort {
    std::string name;
    bool connected = false;
    const OutputPort<T>* source = nullptr;

    explicit InputPort(const std::string& n) : name(n) {}

    std::string_view port_name() const override { return name; }
    const void*      type_id()   const override { return detail::type_tag<T>::id(); }
    bool             is_input()  const override { return true; }
    const void*      read_erased() const override {
        return source ? static_cast<const void*>(&source->value)
                      : static_cast<const void*>(&value_);
    }

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
struct OutputPort : public IPort {
    std::string name;
    T value{};

    explicit OutputPort(const std::string& n) : name(n) {}

    std::string_view port_name() const override { return name; }
    const void*      type_id()   const override { return detail::type_tag<T>::id(); }
    bool             is_input()  const override { return false; }
    const void*      read_erased() const override { return &value; }

    void set(const T& v) { value = v; }
    const T& get() const { return value; }

    // Type-check `in_port` against this output's T, verify it's an input,
    // then perform the typed wiring. Throws on type or direction mismatch.
    void connect_to(IPort& in_port) override {
        if (!in_port.is_input()) {
            detail::invalid_argument(
                "OutputPort::connect_to: target is not an input port");
        }
        if (in_port.type_id() != type_id()) {
            detail::invalid_argument(
                "OutputPort::connect_to: type mismatch between source and destination");
        }
        auto& typed_in = static_cast<InputPort<T>&>(in_port);
        ::shared::connect(*this, typed_in);
    }
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

    // Hook for unit-delay-style blocks: when true, the topological sort
    // ignores this block's *incoming* edges, breaking algebraic loops.
    // Default is false; UnitDelayBlock<T> (shared/blocks/unit_delay.hpp)
    // overrides to true.
    virtual bool is_delay() const { return false; }

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

    // Look up a registered port by its declared name (e.g. "in", "out",
    // "imu", "gnss"). Returns nullptr if not registered. Used by
    // graph-driven wiring where the wiring site doesn't know T.
    IPort* port(std::string_view name) const {
        for (auto* p : ports_) {
            if (p->port_name() == name) return p;
        }
        return nullptr;
    }

protected:
    void mark_updated(uint64_t current_time_us) {
        last_update_time_us_ = current_time_us;
        has_updated_ = true;
    }

    // Register a port by reference so port(name) can find it. Subclasses
    // call this from their ctor; TypedBlock/SourceBlock do it for the
    // ports they own. The Block does not take ownership — port lifetime
    // is tied to the owning block instance.
    void register_port(IPort& p) {
        ports_.push_back(&p);
    }

    std::string name_;
    uint32_t update_period_us_;
    uint64_t last_update_time_us_;
    bool has_updated_;

    std::vector<IPort*> ports_;
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
    {
        this->register_port(input_);
        this->register_port(output_);
    }

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
    {
        this->register_port(output_);
    }

    OutputPort<TOut>& output() { return output_; }
    const OutputPort<TOut>& output() const { return output_; }

protected:
    OutputPort<TOut> output_;
};

} // namespace shared

// CompositeBlock has moved to graph.hpp because it now holds a Graph
// member. Including graph.hpp here keeps backward compat — code that
// said `#include "core/block.hpp"` still gets CompositeBlock.
#include "graph.hpp"
