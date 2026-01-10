#ifndef SHARED_CONTROLLER_H
#define SHARED_CONTROLLER_H

#include "types/state.h"
#include "types/control.h"
#include "types/reference.h"

namespace shared {

/**
 * Abstract controller interface templated on state, reference, and control types.
 * 
 * StateT: Input state type from observer (e.g., StateBase, StateWithBiases)
 * RefT: Reference/setpoint type (e.g., ReferenceBase or sim::Reference)
 * ControlT: Output control type (e.g., ControlEffort<4>)
 * 
 * In sim, concrete controllers also inherit from Block to get scheduling.
 * In embedded, controllers are called directly in control loop.
 */
template<typename StateT, typename RefT, typename ControlT>
class IController {
public:
    virtual ~IController() = default;
    
    /**
     * Set current state estimate from observer.
     */
    virtual void set_state(const StateT& state) = 0;
    
    /**
     * Set reference/setpoint.
     */
    virtual void set_reference(const RefT& ref) = 0;
    
    /**
     * Compute control output.
     * Call after setting state and reference.
     * dt is time since last compute() call [s].
     */
    virtual void compute(double dt) = 0;
    
    /**
     * Get current control output.
     */
    virtual const ControlT& output() const = 0;
    
    /**
     * Reset controller state (integrators, etc.).
     */
    virtual void reset() = 0;
};

/**
 * Base controller implementation with state storage.
 * Provides default implementations for IController interface.
 * 
 * Derived classes override compute() to implement control logic.
 * In sim, also inherit from Block for scheduling.
 */
template<typename StateT, typename RefT, typename ControlT>
class ControllerBase : public IController<StateT, RefT, ControlT> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    ControllerBase() = default;
    virtual ~ControllerBase() = default;
    
    void set_state(const StateT& state) override { state_ = state; }
    void set_reference(const RefT& ref) override { reference_ = ref; }
    const ControlT& output() const override { return control_output_; }
    
    void reset() override {
        control_output_ = ControlT{};
    }
    
    // compute() must be implemented by derived classes
    // void compute(double dt) override;
    
    // Accessors for derived classes
    const StateT& state() const { return state_; }
    const RefT& reference() const { return reference_; }
    
protected:
    StateT state_;
    RefT reference_;
    ControlT control_output_;
};


} // namespace shared

#endif // SHARED_CONTROLLER_H
