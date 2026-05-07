#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include "block.hpp"
#include "misuse.hpp"

namespace shared {

// Generic key-value parameter blob handed to a factory ctor at create time.
// Carries doubles (numeric tunings — PID gains, noise stddevs, etc.) and
// strings (enum-like tags — waveform type, observer variant). Vector / 3-axis
// values are split into _x/_y/_z keys at the call site; struct-of-doubles
// blobs (e.g. EKF noise param sets) flatten the same way. Keys not set fall
// through to the block ctor's compiled defaults.
class BlockParams {
public:
    void set(std::string_view key, double value) {
        doubles_[std::string(key)] = value;
    }
    void set_string(std::string_view key, std::string_view value) {
        strings_[std::string(key)] = std::string(value);
    }

    bool has(std::string_view key) const {
        return doubles_.find(std::string(key)) != doubles_.end()
            || strings_.find(std::string(key)) != strings_.end();
    }

    double get_double(std::string_view key, double default_value = 0.0) const {
        const auto it = doubles_.find(std::string(key));
        return it == doubles_.end() ? default_value : it->second;
    }

    std::string get_string(std::string_view key,
                           const std::string& default_value = "") const {
        const auto it = strings_.find(std::string(key));
        return it == strings_.end() ? default_value : it->second;
    }

private:
    std::unordered_map<std::string, double>      doubles_;
    std::unordered_map<std::string, std::string> strings_;
};

// Factory ctor: takes the block's runtime name (the key for graph wiring,
// e.g. "ekf", "att_ctrl") plus a parameter blob and returns a constructed
// block. Concrete block ctors translate `params` to their typed Params
// struct internally.
using BlockCtor = std::function<std::unique_ptr<Block>(const std::string& block_name,
                                                       const BlockParams&  params)>;

// String → ctor registry. Each block module registers a factory under a
// type name (e.g. "pid", "altitude_hold", "ekf16d"); graph specs name the
// type and the factory produces the Block instance. Instances are
// independent — no global state — so tests build their own factories.
class BlockFactory {
public:
    void register_type(const std::string& type_name, BlockCtor ctor) {
        if (ctors_.find(type_name) != ctors_.end()) {
            detail::invalid_argument(
                "BlockFactory::register_type: duplicate type '" + type_name + "'");
        }
        ctors_.emplace(type_name, std::move(ctor));
    }

    bool has_type(std::string_view type_name) const {
        return ctors_.find(std::string(type_name)) != ctors_.end();
    }

    std::size_t type_count() const { return ctors_.size(); }

    std::unique_ptr<Block> create(const std::string& type_name,
                                  const std::string& block_name,
                                  const BlockParams& params = {}) const {
        const auto it = ctors_.find(type_name);
        if (it == ctors_.end()) {
            detail::invalid_argument(
                "BlockFactory::create: unknown type '" + type_name + "'");
        }
        auto block = it->second(block_name, params);
        // Stash the factory key on the block so the JSON write-back
        // path (dump_graph_json) can emit it without a side-table.
        block->set_type_name(type_name);
        return block;
    }

private:
    std::unordered_map<std::string, BlockCtor> ctors_;
};

} // namespace shared
