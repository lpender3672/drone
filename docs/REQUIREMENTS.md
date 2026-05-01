# Vehicle Control Stack — High-Level Requirements

**Status**: draft · living document
**Last updated**: 2026-05-01

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

Blocks are composable: a `CompositeBlock` owns and wires a set of child blocks and exposes itself as a single `Block` to its parent. The vehicle is a `CompositeBlock` (`shared::QuadrotorVehicle`) holding sensor-driven EKF + outer-loop controllers — the same class runs on sim and embedded. The sim test rig wraps the vehicle in a `QuadrotorPlant` that adds dynamics + sensors + disturbance ports for closed-loop simulation; embedded constructs the vehicle directly and feeds it from real sensor drivers.

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
  ┌──────────────────────────────────────────────────────────────────┐
  │  shared/                  (platform-agnostic — both targets)     │
  │    core/block.hpp            Block, CompositeBlock, ports,       │
  │                              connect(), is_due() scheduling      │
  │    core/interblock_data.hpp  fixed-size array container          │
  │    blocks/pid.hpp            PidBlock                            │
  │    blocks/altitude_hold.hpp  AltitudeHoldBlock                   │
  │    blocks/attitude_controller.hpp  AttitudePidControllerT        │
  │    blocks/quadrotor_ekf.hpp        QuadrotorEkfBlockT            │
  │    blocks/quadrotor_vehicle.hpp    QuadrotorVehicleT (composite) │
  │    data/state.hpp            Scalar, MotorEfforts, …             │
  │    data/gnss_origin.hpp                                          │
  │    types/state.h             TrueState, NavigationState          │
  │    sensors/sensor_readings.h ImuMeasurement, GnssMeasurement, …  │
  │    observer.h                INavObserver interface              │
  │    ekf/shared/ekf16*.{h,cpp} templated INS EKF                   │
  └──────────────────────────────────────────────────────────────────┘
                │                                  │
      ┌─────────▼─────────────────┐    ┌───────────▼──────────────┐
      │  sim/                     │    │  ekf-teensy/             │
      │    QuadrotorPlant         │    │    PlatformIO + Arduino  │
      │      ↳ QuadrotorVehicle   │    │    BNO055/BMP280/uBlox   │
      │      ↳ QuadrotorDynamics  │    │      ↳ QuadrotorVehicle  │
      │      ↳ Sim sensors        │    │    SD logging            │
      │    main.cpp = sensors +   │    │    main.cpp = sensors +  │
      │      Plant + sim.run()    │    │      Vehicle + loop      │
      │    CSV output             │    │                          │
      └───────────────────────────┘    └──────────────────────────┘
```

Data flow on both targets: sensors → vehicle composite (EKF → alt-hold → attitude controller → mixer) → motor commands. In sim, motor commands feed a `QuadrotorDynamics` block whose truth state feeds back into synthetic sensors; on embedded, motor commands eventually drive ESCs and the truth comes from physical sensors. Sensors and actuators are platform-specific; everything between is shared.

### Block composition model

The block runtime is the core abstraction shared across all environments. A `Block` has typed `InputPort<T>` / `OutputPort<T>` members, a declared `update_period_us`, and an `is_due(t)` method. Ports are connected explicitly via `connect(output, input)`, which wires a pointer from input to source — zero-copy, topology-preserving.

A `CompositeBlock` extends `Block` by owning a set of child blocks. Its `update()` iterates children in wiring order and calls `update()` on each due child. From the outside it is opaque — a parent composite (or `Simulation`) sees it as a single block. Children are fixed at construction; runtime add/remove arrives in Tier 2/3 with first-class edges and a `Graph` registry.

The **vehicle composite** (`shared::QuadrotorVehicle`) holds the EKF, alt-hold, and attitude controller as children. It exposes input ports for sensor data (`imu_input`, `gnss_input`, `baro_input`, `mag_input`) and an output port for motor commands. *Sensors are not children of the vehicle* — owners (sim's plant, embedded's main) hold them and `connect()` their outputs into the vehicle. This is what lets the same vehicle class run on both targets unchanged:

```
sim main.cpp:      QuadrotorPlant plant(make_unique<SimIMU>(), make_unique<SimBaro>(), …)
                     // Plant internally constructs a QuadrotorVehicle and
                     // wires sensor outputs / vehicle inputs / dynamics.
embedded main.cpp: QuadrotorVehicle vehicle("quad", std::move(ekf_observer), …);
                     // Sensor outputs are connect()ed into the vehicle directly.
```

The architecture maps directly to Simulink subsystems and makes Tier 2 a natural extension: instead of hardcoding `connect()` calls in the constructor, read them from a graph spec and call the same function.

## 7. Delivery Phases

### Tier 0 — Present state (achieved)
- Shared EKF compiles on host, Teensy, RPi.
- Template-dimensional EKF base allows future estimators of different sizes.
- EKF templated on scalar type so any target that benefits from float vs. double can pick.
- Sim block runtime with runtime observer selection.
- Allan-deviation tuning pipeline emits params automatically.
- Sim produces CSV + estimator-vs-truth plots for regression.

### Tier 1 — Shared vehicle composite (achieved)
Goal: a single vehicle `CompositeBlock` used by both sim and embedded. Changing the controller or wiring edits one file and takes effect on both targets. Both `main.cpp` files slim to sensor construction + composite instantiation.

Work:
1. **Explicit port connections** ✓ — `connect(OutputPort<T>&, InputPort<T>&)` lives in `shared::core::block.hpp`; `Simulation::step()` honours `is_due()`.
2. **`CompositeBlock`** ✓ — `shared::CompositeBlock` owns children and ticks them in wiring order.
3. **Altitude hold controller** ✓ — `shared::AltitudeHoldBlock` cascade (altitude → vertical rate → throttle) feeds the existing attitude controller. Lateral position-hold was removed from scope: not feasible on the target hardware (Teensy 4.1 + standard u-blox GNSS, no RTK or vision aiding) and would just produce drift. If RTK or vision is added later this work item gets re-scoped.
4. **`shared::QuadrotorVehicle` + `sim::QuadrotorPlant`** ✓ — split into a *vehicle* composite (sensors + EKF + alt-hold + attitude controller, runs on both targets) and a sim-only *plant* wrapper (vehicle + dynamics + sim sensors + disturbance ports). Sensors and observer injected as `unique_ptr` at construction; controller and dynamics params taken at construction (no two-phase init). One type universe for sensor and state data across both targets.
5. **Slim both `main.cpp` files** ✓ — each is sensor construction + composite instantiation + a tick loop. No block wiring in main.

Steps 1–2 were infrastructure and unblocked 4–5. Step 3 was scope-reduced to altitude only.

#### What landed alongside Tier 1
- Build system: C++20 across the project; CMake presets pin the generator (MinGW Makefiles on Windows, Unix Makefiles on Linux/Mac) so the same configure works from any shell.
- Tests: consolidated under root `tests/` (was split between `sim/tests/` and ekf's own tree). 49 unit tests cover ports, scheduling, PID, alt-hold, EKF block timestamp gating, dynamics, controller. CI runs them on every push.
- Embedded: shared block runtime moved out of `sim::` into `shared::`; sensors decoupled from the EKF observer pointer; embedded `main.cpp` now uses the same `QuadrotorVehicle` composite as sim.

### Tier 2 — Graph-driven composition
Goal: headless Simulink. Sim reads a YAML/JSON graph spec, instantiates blocks, wires them, runs. A/B compare EKF variants without recompiling.

Tier 1 makes this a natural extension — the `connect()` calls currently hardcoded in the vehicle constructor are read from a graph spec instead. The block infrastructure is unchanged.

#### Already in place from Tier 1
- `shared::CompositeBlock` is the natural target for graph-driven children — same class, just children added from a spec instead of a constructor.
- `connect(out, in)` is the canonical wiring primitive.
- Sensor and state types unified — Tier 2's port type-erasure builds on a single type universe, not two.
- `gtest_discover_tests` plus the root-level `tests/` tree means new graph-related blocks get test coverage in the same loop.

#### Work
1. **First-class edges and a `Graph` registry.** Today `connect(out, in)` writes a raw pointer into `in.source`; nothing tracks the edge. A `Graph` (or `BlockRegistry`) that owns the canonical block-and-edge list is the foundation for both Tier 2 graph specs and Tier 3 mutation. With it, you can ask "what's downstream of this block?" before deleting it, do topological sort, and write a graph back out as YAML.
2. **Type-erased port base** so `connect(port_a, port_b)` is callable without knowing T at the wiring site. Likely a runtime type tag plus a pool keyed by (block, port-name). Has to coexist with the existing template-typed `connect()` for the C++ wiring sites.
3. **Block factory registry** mapping string names to constructors: `"ekf16d"` → `std::make_unique<EKF16d>(params)`, `"altitude_hold"` → `std::make_unique<AltitudeHoldBlock<>>(...)`. Each block module registers itself.
4. **Topological sort** replacing hardcoded child ordering in `CompositeBlock`. Runs once after the graph is loaded; child ordering becomes a cached output of the sort.
5. **Unit-delay blocks** for explicit algebraic-loop resolution. The current child-ordering trick assumes no algebraic loops; once topology is data-driven we need a way to break loops explicitly.
6. **Signal-trace auto-logging.** Any port can be tagged in the graph spec; tagged ports get logged automatically. This replaces the dead `IInterBlockData` / `DataLogger` infrastructure left in the codebase from earlier — it should be deleted as part of this work item, not left lying around.

#### Expected deliverable
A `quad.yaml` (or `.json`) that produces the same closed-loop behaviour as the current hardcoded `QuadrotorPlant` ctor, plus an `ekf_compare.yaml` that swaps the EKF for an alternative observer with no recompile.

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
- Is the controller's PID tuning generic enough for the shared vehicle, or do sim and flight actually want different gains (e.g. because of unmodelled actuator dynamics)? Open — same gains shipped on both targets in Tier 1; first hardware bring-up will tell us.
- Alt-hold tuning: the default `AltitudeHoldBlock` gains keep altitude within a few metres over a 2-minute hover but exhibit a slow oscillation under the current sim dynamics. Worth a tuning pass before flight; also a candidate for the first real long-duration integration test (assert "altitude stays within ±2 m of setpoint over 60 s").

### Resolved during Tier 1
- ~~`CompositeBlock::update()` calls children in wiring order — sufficient or need dynamic re-ordering?~~ Fixed wiring order is fine for Tier 1; Tier 2's topological sort will replace it.
- ~~Sensor data and state types: should sim wrap them with logger sub-data, or use shared types directly?~~ Unified on shared types. Per-vehicle state extensions (fixed-wing airspeed, helicopter rotor RPM) will derive from the shared base when needed; quadrotor doesn't need any extension.

## 9. Document history
- **2026-04-24** — initial draft, captures current state and Tier 0–3 roadmap.
- **2026-04-24** — generalise scope from "drone" to any vehicle (flying types primary); drop standalone scalar-precision FR (kept as an implementation detail of FR-3, not a top-level requirement); add FR-4 vehicle-agnostic core.
- **2026-04-26** — Tier 0 confirmed complete. Revised Tier 1: replace "sensor pack trait + factory template" approach with composite block / dependency injection model. A vehicle is a `CompositeBlock` that accepts sensor blocks by injection — no template metaprogramming, direct path to Tier 2 graph-driven wiring. Explicit `connect()` and `CompositeBlock` added as infrastructure prerequisites. Updated FR-1 and architecture section accordingly.
- **2026-05-01** — Tier 1 closed out. `QuadrotorSystem` split into `shared::QuadrotorVehicle` (cross-target composite) + `sim::QuadrotorPlant` (sim wrapper adding dynamics + sim sensors + disturbance ports). Block runtime + concrete blocks (PID, alt-hold, attitude controller, EKF block) moved into `shared/`; one type universe for sensor and state data; sensors decoupled from observer; embedded `main.cpp` slimmed to use the same vehicle composite. Position-hold removed from scope (altitude-only). Test infrastructure consolidated under root `tests/`, mocks in `tests/mocks/`, 49 unit tests with CI gating. Tier 2 work list refined with concrete first steps (first-class edges + Graph registry as the foundation).
