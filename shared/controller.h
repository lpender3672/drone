#ifndef SHARED_CONTROLLER_H
#define SHARED_CONTROLLER_H

namespace shared {

/**
 * Abstract controller interface templated on state, reference, and control types.
 * 
 * StateT: Input state type from observer (e.g., StateBase, NavigationState)
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

} // namespace shared

#endif // SHARED_CONTROLLER_H
