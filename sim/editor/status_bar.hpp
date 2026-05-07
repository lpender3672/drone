#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace editor {

enum class StatusLevel {
    Info,
    Warning,
    Error,
};

struct StatusMessage {
    StatusLevel level;
    std::string message;
    uint64_t    posted_us;
};

// Holds the latest user-visible status message with a posted-time stamp.
// The render layer (main.cpp's bottom-pinned window) reads `current(now)`
// each frame; messages fade out after `ttl_us`. A new `post()` always
// replaces the previous message — no queueing, single-line bar.
//
// Slice 3 uses one TTL across all levels; if errors should later stick
// until dismissed, that policy lives here.
class StatusBar {
public:
    explicit StatusBar(uint64_t ttl_us = 5'000'000) : ttl_us_(ttl_us) {
        if (ttl_us == 0) {
            throw std::invalid_argument(
                "StatusBar: ttl_us must be > 0 (use clear() to remove a message)");
        }
    }

    void post(StatusLevel level, std::string message, uint64_t now_us) {
        current_ = StatusMessage{level, std::move(message), now_us};
    }

    std::optional<StatusMessage> current(uint64_t now_us) const {
        if (!current_) return std::nullopt;
        if (now_us > current_->posted_us &&
            (now_us - current_->posted_us) > ttl_us_) {
            return std::nullopt;
        }
        return current_;
    }

    void clear() { current_.reset(); }

    uint64_t ttl_us() const { return ttl_us_; }

private:
    uint64_t                     ttl_us_;
    std::optional<StatusMessage> current_;
};

} // namespace editor
