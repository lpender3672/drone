#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include <core/block_factory.hpp>
#include <core/graph.hpp>
#include <core/graph_loader.hpp>

#include "recent_files.hpp"
#include "status_bar.hpp"

namespace editor {

// AppState owns the editor's "document" — the loaded graph + the path
// it came from + the related UI state (status bar message, recent
// files list). The render layer in main.cpp holds one of these and
// reads accessors per frame; menu actions call methods on it.
//
// The factory is registered up-front (sim::register_quadrotor_blocks)
// and handed in by value; reloads don't re-register, and tests use a
// minimal factory with one block type to keep AppState's unit narrow.
//
// Failure semantics are *atomic*: a failed `load_graph` (missing file,
// malformed JSON, unknown block type, bad edge) leaves the prior graph
// and current_path untouched — partial state would be the worst UX.
// Errors land as a StatusLevel::Error in the status bar; success
// posts a one-line "loaded N blocks, M edges" Info.
class AppState {
public:
    explicit AppState(shared::BlockFactory factory)
        : factory_(std::move(factory)) {}

    // Read `path`, parse it, swap into graph_ on success. now_us
    // timestamps the status message for fade-out.
    bool load_graph(const std::string& path, uint64_t now_us) {
        std::ifstream in(path);
        if (!in) {
            status_.post(StatusLevel::Error,
                         "could not read '" + path + "'", now_us);
            return false;
        }
        std::ostringstream buf;
        buf << in.rdbuf();

        shared::Graph fresh;
        try {
            shared::load_graph_json(buf.str(), factory_, fresh);
        } catch (const std::exception& e) {
            status_.post(StatusLevel::Error,
                         "load failed: " + std::string(e.what()), now_us);
            return false;
        }

        graph_        = std::move(fresh);
        current_path_ = path;
        recent_.push(path);
        status_.post(StatusLevel::Info,
                     "loaded " + path + " (" +
                         std::to_string(graph_.block_count()) + " blocks, " +
                         std::to_string(graph_.edge_count())  + " edges)",
                     now_us);
        return true;
    }

    // Re-read the most-recently-loaded path. No-op (returns false)
    // when nothing has been loaded yet.
    bool reload(uint64_t now_us) {
        if (current_path_.empty()) return false;
        return load_graph(current_path_, now_us);
    }

    // Write the current graph to `path` as JSON. Updates current_path
    // and pushes to recent on success — same convention as load_graph.
    // Slice 4 contract: topology only (params don't round-trip yet).
    bool save_graph(const std::string& path, uint64_t now_us) {
        std::string text;
        try {
            text = shared::dump_graph_json(graph_);
        } catch (const std::exception& e) {
            status_.post(StatusLevel::Error,
                         "save failed: " + std::string(e.what()), now_us);
            return false;
        }

        std::ofstream out(path);
        if (!out) {
            status_.post(StatusLevel::Error,
                         "could not write '" + path + "'", now_us);
            return false;
        }
        out << text;
        if (!out) {
            status_.post(StatusLevel::Error,
                         "write failed for '" + path + "'", now_us);
            return false;
        }

        current_path_ = path;
        recent_.push(path);
        status_.post(StatusLevel::Info,
                     "saved " + path + " (" +
                         std::to_string(graph_.block_count()) + " blocks, " +
                         std::to_string(graph_.edge_count())  + " edges)",
                     now_us);
        return true;
    }

    // Drop the open document. Recent-files survives — the user may
    // want to reopen something they just closed.
    void clear() {
        graph_ = shared::Graph{};
        current_path_.clear();
    }

    const shared::Graph& graph()         const { return graph_; }
    const std::string&   current_path()  const { return current_path_; }
    const StatusBar&     status_bar()    const { return status_; }
    const RecentFiles&   recent_files()  const { return recent_; }

    // Mutable graph access for the canvas — `set_graph(&app.graph_mut())`
    // since CanvasView holds a raw pointer that survives load swaps.
    shared::Graph& graph_mut() { return graph_; }

    // Mutable status bar so the toolbar (and other UI surfaces in main.cpp)
    // can post their own messages without going through AppState's load
    // path. AppState's accessors are const-by-default for cheap reads;
    // these `_mut` siblings are the explicit "I'm going to write" handle.
    StatusBar& status_bar_mut() { return status_; }

private:
    shared::BlockFactory factory_;
    shared::Graph        graph_;
    std::string          current_path_;
    StatusBar            status_;
    RecentFiles          recent_;
};

} // namespace editor
