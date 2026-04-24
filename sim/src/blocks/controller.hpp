#pragma once

#include "../core/block.hpp"
#include "../../shared/controller.h"

namespace sim {

template<typename NavigationStateT, typename ReferenceT, typename EffortsT>
class ControllerBlock : public Block {
public:
    using StateType = NavigationStateT;
    using ReferenceType = ReferenceT;
    using OutputType = EffortsT;
    
    ControllerBlock(const std::string& name,
                    const std::string& state_input_name,
                    const std::string& reference_input_name,
                    const std::string& output_name,
                    uint32_t update_period_us)
        : Block(name, update_period_us)
        , state_input_(state_input_name)
        , reference_input_(reference_input_name)
        , output_(output_name)
    {}
    
    InputPort<NavigationStateT>& state_input() { return state_input_; }
    const InputPort<NavigationStateT>& state_input() const { return state_input_; }
    
    InputPort<ReferenceT>& reference_input() { return reference_input_; }
    const InputPort<ReferenceT>& reference_input() const { return reference_input_; }
    
    OutputPort<EffortsT>& output() { return output_; }
    const OutputPort<EffortsT>& output() const { return output_; }
    
    virtual void reset() = 0;

protected:
    InputPort<NavigationStateT> state_input_;
    InputPort<ReferenceT> reference_input_;
    OutputPort<EffortsT> output_;
};

} // namespace sim