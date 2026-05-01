#pragma once

#include <core/block.hpp>

namespace mocks {

// Minimal Block test double — one typed input port, one typed output port,
// passthrough update(). Kept template-on-T so tests can pick whatever
// signal type they want; defaults to double.
template<typename T = double>
class MockBlock : public shared::Block {
public:
    explicit MockBlock(const std::string& name, uint32_t update_period_us = 0u)
        : Block(name, update_period_us)
        , in_("in")
        , out_("out")
    {
        this->register_port(in_);
        this->register_port(out_);
    }

    bool update(uint64_t t) override {
        if (!is_due(t)) return false;
        out_.value = in_.get();
        mark_updated(t);
        ++update_count;
        return true;
    }

    shared::InputPort<T>&  in()  { return in_; }
    shared::OutputPort<T>& out() { return out_; }

    int update_count = 0;

private:
    shared::InputPort<T>  in_;
    shared::OutputPort<T> out_;
};

} // namespace mocks
