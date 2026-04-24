# Vehicle Control Stack — High-Level Requirements

**Status**: draft · living document
**Last updated**: 2026-04-24

## 1. Purpose & Vision

A unified C++ framework where a vehicle's estimation and control algorithms are authored **once** as composable blocks, then run in two environments with no code duplication:

- **Host simulation** — closed-loop test against a physics model, with synthetic sensors, for regression testing and tuning.
- **Embedded deployment** — the same algorithm blocks deployed to a microcontroller, wired to real sensor drivers.

The framework is vehicle-agnostic by design. Primary focus is **flying vehicles of any configuration** — multirotors (quad/hex/octo), fixed-wing, helicopters, VTOL hybrids, airships — with the architecture explicitly not precluding ground vehicles, surface/underwater craft, or spacecraft. The current implementation targets a quadrotor; adding a new vehicle type means providing its dynamics model, sensor set, and controller while reusing the same navigation filter, block runtime, and tuning pipeline.

The long-term north star is a Simulink-style block diagram that can be edited, simulated, and cross-compiled to firmware. The short-term priority is removing duplication and locking in reproducibility.

## 2. Scope

### In scope
- Inertial Navigation EKF (position, velocity, attitude, sensor biases) as the default estimator, with architecture allowing simpler estimators (AHRS-only, complementary filter) for small vehicles.
- Attitude and position/altitude control; vehicle-specific mixers (quadrotor motor mixer, fixed-wing control-surface mixer, etc.) as pluggable blocks.
- Sensor drivers: IMU, GNSS, barometer, magnetometer, with room for airspeed, range finder, optical flow, etc. as additional sensor types are needed.
- Sim-side physics: 6-DoF rigid-body dynamics common to any vehicle, plus vehicle-specific force/moment models (rotor thrust, fixed-wing aero, etc.).
- Sim-side synthetic sensor models matching real-sensor noise characteristics.
- Parameter tuning from static slab data (Allan-deviation pipeline, already implemented).
- SD-card logging on embedded; CSV export in sim.

### Current implementation
Quadrotor. First secondary vehicle type will likely be fixed-wing.

### Out of scope (for now)
- Visual block-diagram editor (aspirational — Tier 3 below).
- Hybrid/continuous-time solvers (everything is fixed-step discrete).
- Automatic code generation from graph specs.
- Multi-vehicle simulation (one vehicle per sim instance).
- Swarm / inter-vehicle communication.

## 3. Stakeholders

- **Developer** (primary): designs, simulates, tunes, flashes.
- **Future collaborators**: should be able to swap a block (e.g. a controller) without understanding the full stack.

## 4. Functional Requirements

### FR-1: Block composition
The system shall express estimation and control as a directed graph of blocks with typed input and output ports. Each block has a declared update period and self-schedules via `is_due(t)`.

### FR-2: Swappable estimator
Sim shall allow the navigation filter to be selected at run-time (e.g. EKF16d vs. a future AHRS-7 or tight-coupled variant) without recompiling the block wiring. Achieved via `std::unique_ptr<INavObserver>` injection.

### FR-3: Shared algorithm code
Estimator, controller, and reference-trajectory blocks shall build and run identically on host (sim) and on the embedded target. Only sensor drivers, actuator drivers, and the top-level wiring are platform-specific.

### FR-4: Vehicle-agnostic core
The navigation filter, block runtime, tuning pipeline, logging, and signal-capture tools shall have no vehicle-specific assumptions. Vehicle-specific code (dynamics model, mixer, controller gains, sensor set) lives in a vehicle module that plugs into the shared core.

### FR-5: Parameter flow
Filter tuning parameters (process/measurement noise, gravity-aiding sigmas) shall come from two sources:
  - `tuned_ekf_params.h` — auto-generated from `tune.py` Allan-deviation analysis of static slab logs.
  - `sim_ekf_params.h` — hand-tuned override for sim (synthetic noise doesn't need Allan analysis).

### FR-6: Sensor logging for tuning
The embedded build shall log raw IMU/baro/mag streams to SD via an `ILogger` interface decoupled from the drivers. The logs shall be consumable by `tune.py` to regenerate filter parameters.

### FR-7: Signal capture in sim
The sim shall write per-timestep CSV of ground truth and estimator output suitable for plotting.

### FR-8: Deterministic replay
Given fixed RNG seeds, the sim shall produce byte-identical output across runs to enable regression testing.

## 5. Non-Functional Requirements

### NFR-1: No embedded heap in hot path
Embedded builds shall set `EIGEN_NO_MALLOC`. All matrices used in `predict()`/`update_*()` shall be fixed-size and stack-allocated (via `BufferReserverT`).

### NFR-2: Real-time on 1 kHz loop
Predict step + one measurement update shall complete in under 500 µs on a Teensy 4.1 (Cortex-M7 @ 600 MHz) and under 1 ms on an STM32H7 (Cortex-M7 @ 400 MHz) with single-precision FPU.

### NFR-3: Portability
All shared code shall compile clean on: host GCC/Clang (Linux, macOS), Arduino toolchain (Teensy 4.1), and ARM GCC for STM32. No platform-specific intrinsics in algorithm code.

### NFR-4: Binary size
Embedded firmware shall fit in under 512 KB flash on the primary target. Template instantiations that aren't used on a given target shall be omittable via build flag.

### NFR-5: Numerical correctness
The sim's estimator output shall track ground truth with bounded error on a stable flight profile. Specific bounds per vehicle type (e.g. attitude error < 2° RMS during hover for a quadrotor) are documented in the vehicle module, not the core.

## 6. Architecture Overview

```
  ┌─────────────────────────────────────────────────────────┐
  │  shared/                   (platform-agnostic algos)    │
  │    ekf16.h/cpp             templated INS EKF core       │
  │    ekf16d, ekf16f          scalar-specific typedefs     │
  │    observer.h              INavObserver interface       │
  │    types/state.h           NavigationState, TrueState   │
  │    sensors/*.h             sensor I/O interfaces        │
  └─────────────────────────────────────────────────────────┘
                │                              │
      ┌─────────▼─────────┐          ┌─────────▼──────────┐
      │  sim/             │          │  ekf-teensy/       │
      │    Block runtime  │          │    PlatformIO      │
      │    SimSensors     │          │    HwDrivers       │
      │    Dynamics       │          │    SD logging      │
      │    CSV output     │          │                    │
      └───────────────────┘          └────────────────────┘

      ┌─────────────────────────┐    ┌────────────────────┐
      │  ekf/rpi/               │    │  future: stm32/    │
      │    Raspberry Pi test    │    │    CubeMX project  │
      │    INS runner           │    │    EKF16<float>    │
      └─────────────────────────┘    └────────────────────┘
```

Data flow in all environments: sensors → observer (EKF) → controller → actuators / dynamics. Sensors and actuators are platform-specific; everything between is shared.

## 7. Delivery Phases

### Tier 0 — Present state (achieved)
- Shared EKF compiles on host, Teensy, RPi.
- Template-dimensional EKF base allows future estimators of different sizes.
- EKF templated on scalar type so any target that benefits from float vs. double can pick.
- Sim block runtime with runtime observer selection.
- Allan-deviation tuning pipeline emits params automatically.
- Sim produces CSV + estimator-vs-truth plots for regression.

### Tier 1 — Shared vehicle system description
Goal: a single vehicle-system spec used by both sim and embedded top-level mains. Changing the controller or wiring edits one file and takes effect on both targets.

Work:
- Vehicle-module interface (dynamics, mixer, default controller, sensor set) so a new vehicle type means implementing this interface once.
- Template over a "sensor pack" trait so sim uses synthetic sensors and embedded uses real drivers, with the same system wiring.
- Move the block-wiring logic out of each `main.cpp` into a shared `construct_system<Vehicle>()` factory.
- Add altitude/position hold controllers (currently missing; quadrotor sinks in sim).

### Tier 2 — Graph-driven composition
Goal: headless Simulink. Sim reads a YAML/JSON graph spec, instantiates blocks, wires them, runs. A/B compare EKF variants without recompiling.

Work:
- Type-erased port base (or runtime-typed port pool) so `connect(port_a, port_b)` is a legal operation.
- Block factory registry (`"ekf16d"` → `std::make_unique<EKF16d>(params)`).
- Topological sort + automatic scheduling in `Simulation`.
- Unit-delay blocks for explicit cycle resolution.
- Signal-trace auto-logging (tag any port, it gets logged).

### Tier 3 — Visual editor + codegen (long-term)
Goal: edit a block diagram in a UI, simulate it, then flash the same graph to the STM32.

Work:
- Electron/Tauri front-end with React Flow (or similar) for the graph editor.
- Graph spec → YAML persistence.
- Graph spec → C++ template emission for the embedded target (`step()` function + static block instances). No generic C codegen — emits against the existing block API.
- Build integration: graph.yaml becomes a CMake input; changes trigger a re-emit.

## 8. Risks & Open Questions

### Risks
- **Template instantiation bloat** on resource-constrained MCUs. Mitigation: gate unused instantiations (e.g. `EKF16<float>` on Teensy) behind build flags.
- **Visual editor scope** — Tier 3 is months of work and easy to underestimate. Mitigation: Tier 2 headless graph is useful on its own and stops here is a fine outcome.
- **Gravity-aiding robustness** during real flight maneuvers. Current magnitude gate may not be enough under aggressive dynamics. Follow-up: GNSS-derived `a_kin` gating once Tier 1 is done.

### Open questions
- Does the STM32 target need a real-time OS (FreeRTOS) or is a bare-metal tick sufficient? Current thinking: bare-metal for initial bring-up, RTOS if we need concurrent tasks (e.g. radio RX + control + logging).
- What granularity of reproducibility do we need? Bit-exact replay across host platforms would require fixed-point RNG or seed-per-block; currently seed-per-process is enough for regression.
- Is the controller's PID tuning generic enough for Tier 1's shared system, or do sim and flight actually want different gains (e.g. because of unmodelled actuator dynamics)?
- How much of the quadrotor-specific code under `sim/src/quadcopter/` genuinely belongs in a vehicle module vs. the shared core? Likely: dynamics + mixer are vehicle-specific; the block scheduling, state types, and estimator wiring are not.

## 9. Document history
- **2026-04-24** — initial draft, captures current state and Tier 0–3 roadmap.
- **2026-04-24** — generalise scope from "drone" to any vehicle (flying types primary); drop standalone scalar-precision FR (kept as an implementation detail of FR-3, not a top-level requirement); add FR-4 vehicle-agnostic core.
