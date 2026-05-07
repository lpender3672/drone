#pragma once

#include <stdexcept>

namespace editor {

enum class RunPhase {
    Stopped,
    Running,
    Paused,
};

// Editor's Run / Pause / Reset state machine.
//
// Slice 3 stubs the actual sim ticking — the toolbar buttons just flip
// this enum (and post a status). Slice 6 hooks the editor's tick loop:
// while `is_running()` is true, walk Graph::topo_order() once per frame
// (or per fixed step), respecting `is_paused()`.
//
// Pause from Stopped or Paused is treated as a programming error: the
// UI layer should disable the Pause button when not Running, so a call
// here means a wiring bug. Run is idempotent (double-click forgiven).
class RunState {
public:
    RunPhase current() const { return phase_; }

    bool is_running() const { return phase_ == RunPhase::Running; }
    bool is_paused()  const { return phase_ == RunPhase::Paused;  }
    bool is_stopped() const { return phase_ == RunPhase::Stopped; }

    void run() {
        // Stopped → Running (start), Paused → Running (resume).
        // Running → Running is a no-op so a double-click doesn't throw.
        phase_ = RunPhase::Running;
    }

    void pause() {
        if (phase_ != RunPhase::Running) {
            throw std::logic_error(
                "RunState::pause: only valid while running");
        }
        phase_ = RunPhase::Paused;
    }

    void reset() {
        phase_ = RunPhase::Stopped;
    }

private:
    RunPhase phase_ = RunPhase::Stopped;
};

} // namespace editor
