#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace editor {

// Bounded most-recent-first list of recently-opened file paths.
// `push(path)` adds at the front; if `path` already exists, it gets
// moved to the front instead of duplicated. Past the cap, the oldest
// entry is evicted. Drives the editor's File > Recent submenu.
//
// Pure logic — no filesystem access. The menu layer in main.cpp reads
// `entries()` each frame and emits a MenuItem per entry; the chosen
// entry feeds back through AppState::load_graph.
class RecentFiles {
public:
    explicit RecentFiles(std::size_t max = 8) : max_(max) {
        if (max == 0) {
            throw std::invalid_argument("RecentFiles: max_size must be ≥ 1");
        }
    }

    void push(const std::string& path) {
        if (path.empty()) {
            throw std::invalid_argument("RecentFiles::push: path is empty");
        }
        // Move-to-front if already present — the user re-opening the
        // same file shouldn't grow the list or push other entries down.
        const auto it = std::find(entries_.begin(), entries_.end(), path);
        if (it != entries_.end()) {
            entries_.erase(it);
        }
        entries_.insert(entries_.begin(), path);
        if (entries_.size() > max_) {
            entries_.resize(max_);
        }
    }

    void clear() { entries_.clear(); }

    const std::vector<std::string>& entries() const { return entries_; }

    std::size_t size()     const { return entries_.size(); }
    std::size_t max_size() const { return max_; }

private:
    std::vector<std::string> entries_;
    std::size_t              max_;
};

} // namespace editor
