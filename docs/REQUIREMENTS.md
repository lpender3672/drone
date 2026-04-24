# Drone Flight Stack — High-Level Requirements

**Status**: draft · living document
**Last updated**: 2026-04-24

## 1. Purpose & Vision

A unified C++ framework where a drone's estimation and control algorithms are authored **once** as composable blocks, then run in two environments with no code duplication:

- **Host simulation** — closed-loop test against a physics model, with synthetic sensors, for regression testing and tuning.
- **Embedded flight** — the same algorithm blocks deployed to an STM32-class microcontroller, wired to real sensor drivers.

The long-term north star is a Simulink-style block diagram that can be edited, simulated, and cross-compiled to firmware. The short-term priority is removing duplication and locking in reproducibility.

## 2. Scope

### In scope
- Inertial Navigation EKF (position, velocity, attitude, sensor biases)
- Attitude and (eventually) position/altitude control
- Sensor drivers: IMU, GNSS, barometer, magnetometer
- Sim-side physics: quadrotor 6-DoF dynamics, motor/prop models, disturbances
- Sim-side synthetic sensor models matching real-sensor noise characteristics
- Parameter tuning from static slab data (Allan-deviation pipeline, already implemented)
- SD-card logging on embedded; CSV export in sim

### Out of scope (for now)
- Visual block-diagram editor (aspirational — Tier 3 below)
- Hybrid/continuous-time solvers (everything is fixed-step discrete)
- Automatic code generation from graph specs
- Non-quadrotor platforms
- Multi-vehicle simulation

## 3. Stakeholders

- **Developer** (primary): designs, simulates, tunes, flashes.
- **Future collaborators**: should be able to swap a block (e.g. a controller) without understanding the full stack.

## 4. Functional Requirements

### FR-1: Block composition
The system shall express estimation and control as a directed graph of blocks with typed input and output ports. Each block has a declared update period and self-schedules via `is_due(t)`.

### FR-2: Swappable estimator
Sim shall allow the navigation filter to be selected at run-time (e.g. EKF16d vs. a future AHRS-7 or tight-coupled variant) without recompiling the block wiring. Achieved via `std::unique_ptr<INavObserver>` injection.

### FR-3: Shared algorithm code
EKF, controller, and reference trajectory blocks shall build and run identically on host (sim) and on the embedded target. Only sensor drivers and the top-level wiring are platform-specific.

### FR-4: Scalar precision
The EKF shall be templated on the scalar type (`double` / `float`) so that STM32 targets with single-precision FPU can use `EKF16<float>` without re-implementation. Already implemented (see `ekf16.h`).

### FR-5: Parameter flow
Filter tuning parameters (process/measurement noise, gravity-aiding sigmas) shall come from two sources:
  - `tuned_ekf_params.h` — auto-generated from `tune.py` Allan-deviation analysis of static slab logs
  - `sim_ekf_params.h` — hand-tuned override for sim (synthetic noise doesn't need Allan analysis)

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
Float-scalar EKF shall produce results within 1% of the double-scalar version over 60 s of sim data. Larger drift is a bug, not an acceptable float-precision consequence.

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
- Scalar-templated EKF (EKF16d for host/Teensy, EKF16f for STM32).
- Sim block runtime with runtime observer selection.
- Allan-deviation tuning pipeline emits params automatically.
- Sim produces CSV + EKF-vs-truth plots for regression.

### Tier 1 — Shared system description
Goal: a single "quadrotor system" spec used by both sim and Teensy top-level mains. Changing the controller or wiring edits one file, flies on both.

Work:
- Template over a "sensor pack" trait so sim uses synthetic sensors and Teensy uses real drivers, with the same `QuadrotorSystem` wiring.
- Move the block-wiring logic out of each `main.cpp` into a shared `construct_quadrotor_system()` factory.
- Add altitude hold controller (currently missing; drone sinks in sim).

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
- Is the controller's PID tuning generic enough for Tier 1's shared system, or do sim and flight actually want different gains (e.g. because of unmodelled motor dynamics)?

## 9. Document history
- **2026-04-24** — initial draft, captures current state and Tier 0–3 roadmap.
