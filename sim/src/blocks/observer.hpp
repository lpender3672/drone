#pragma once

#include "../core/block.hpp"
#include "../../shared/observer.h"

namespace sim {

template<typename SensedStateT, typename NavigationStateT>
class ObserverBlock : public TypedBlock<SensedStateT, NavigationStateT> {
public:
    ObserverBlock(const std::string& name,
                  std::unique_ptr<shared::IObserver<NavigationStateT>> observer,
                  uint32_t update_period_us)
        : TypedBlock<SensedStateT, NavigationStateT>(name, "sensors", "state", update_period_us)
        , observer_(std::move(observer))
    {}

    bool update(uint64_t current_time_us) override {
        if (!this->is_due(current_time_us)) return false;
        this->mark_updated(current_time_us);

        feed_sensors(this->input_.value);
        this->output_.value = observer_->output();
        return true;
    }

    void reset(const NavigationStateT& initial) {
        observer_->reset(initial);
    }

    shared::IObserver<NavigationStateT>& observer() { return *observer_; }
protected:
    virtual void feed_sensors(const SensedStateT& sensors) = 0;

    std::unique_ptr<shared::IObserver<NavigationStateT>> observer_;
};

} // namespace sim