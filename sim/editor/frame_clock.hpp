#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace editor {

// Pure frame-pacing counter for the editor render loop. Time-source
// agnostic — `tick(now_us)` is called once per rendered frame with a
// monotonic timestamp (e.g. GLFW's `glfwGetTime() * 1e6`); the clock
// records the count and the most recent inter-frame interval.
//
// First tick has no prior interval, so `dt_ms()` stays 0 — matches the
// `shared::Block` first-update convention. `fps()` is a convenience
// derived from `dt_ms()`.
//
// Header-only. No GLFW / OpenGL / ImGui dependency — testable as pure
// logic; the editor's main loop wires GLFW's clock in.
class FrameClock {
public:
    void tick(uint64_t now_us) {
        if (frame_count_ > 0 && now_us < last_us_) {
            throw std::invalid_argument(
                "FrameClock::tick: timestamp went backwards (" +
                std::to_string(now_us) + " < " +
                std::to_string(last_us_) + ")");
        }
        if (frame_count_ == 0) {
            dt_ms_ = 0.0;
        } else {
            dt_ms_ = static_cast<double>(now_us - last_us_) * 1e-3;
        }
        last_us_ = now_us;
        ++frame_count_;
    }

    void reset() {
        frame_count_ = 0;
        last_us_     = 0;
        dt_ms_       = 0.0;
    }

    uint64_t frame_count() const { return frame_count_; }
    double   dt_ms()       const { return dt_ms_; }
    double   fps()         const { return dt_ms_ > 0.0 ? 1000.0 / dt_ms_ : 0.0; }

private:
    uint64_t frame_count_ = 0;
    uint64_t last_us_     = 0;
    double   dt_ms_       = 0.0;
};

} // namespace editor
