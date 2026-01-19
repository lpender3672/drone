#pragma once

#include "../core/block.hpp"
#include "../../shared/observer.h"

namespace sim {

template<typename SensedStateT, typename ObservedStateT>
class ObserverBlock : public TypedBlock<SensedStateT, ObservedStateT> {
public:
    ObserverBlock(const std::string& name,
                  std::unique_ptr<shared::IObserver<ObservedStateT>> observer,
                  double update_rate_hz)
        : TypedBlock<SensedStateT, ObservedStateT>(name, "sensors", "state", update_rate_hz)
        , observer_(std::move(observer))
    {}

    bool update(double current_time_s) override {
        if (!this->is_due(current_time_s)) return false;
        this->mark_updated(current_time_s);

        feed_sensors(this->input_.value);
        this->output_.value = observer_->output();
        return true;
    }

    void reset(const ObservedStateT& initial) {
        observer_->reset(initial);
    }

    shared::IObserver<ObservedStateT>& observer() { return *observer_; }
protected:
    virtual void feed_sensors(const SensedStateT& sensors) = 0;

    std::unique_ptr<shared::IObserver<ObservedStateT>> observer_;
};

} // namespace sim