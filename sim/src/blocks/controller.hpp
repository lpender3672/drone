#pragma once

#include "../core/block.hpp"
#include "../../shared/controller.h"

namespace sim {

// Input bundle for controllers
template<typename ObservedStateT, typename ReferenceT>
struct ControllerInput {
    ObservedStateT state{};
    ReferenceT reference{};
};

template<typename ObservedStateT, typename ReferenceT, typename EffortsT>
class ControllerBlock : public TypedBlock<ControllerInput<ObservedStateT, ReferenceT>, EffortsT> {
public:
    using InputT = ControllerInput<ObservedStateT, ReferenceT>;
    using Base = TypedBlock<InputT, EffortsT>;

    ControllerBlock(const std::string& name,
                    std::unique_ptr<shared::IController<ObservedStateT, ReferenceT, EffortsT>> controller,
                    double update_rate_hz)
        : Base(name, "state_ref", "efforts", update_rate_hz)
        , controller_(std::move(controller))
    {}

    bool update(double current_time_s) override {
        if (!this->is_due(current_time_s)) return false;
        
        double dt = this->update_period_s_;
        if (this->last_update_time_s_ > 0) {
            dt = current_time_s - this->last_update_time_s_;
        }
        
        this->mark_updated(current_time_s);

        const auto& in = this->input_.value;
        controller_->set_state(in.state);
        controller_->set_reference(in.reference);
        controller_->compute(dt);
        
        this->output_.value = controller_->output();
        return true;
    }

    void reset() {
        controller_->reset();
    }

    // Direct access if needed
    shared::IController<ObservedStateT, ReferenceT, EffortsT>& controller() { 
        return *controller_; 
    }

protected:
    std::unique_ptr<shared::IController<ObservedStateT, ReferenceT, EffortsT>> controller_;
};

} // namespace sim