#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>

#include "block.hpp"

namespace shared {

// Generic key-value parameter blob handed to a factory ctor at create time.
// Slice 3 carries doubles only (covers PID gains, alt-hold gains, etc.);
// extend with strings / vectors when a block type needs them. Keeping it
// minimal and homogeneous makes YAML deserialisation in a later slice
// trivial — keys map straight from the YAML node.
class BlockParams {
public:
    void set(std::string_view key, double value) {
        values_[std::string(key)] = value;
    }

    bool has(std::string_view key) const {
        return values_.find(std::string(key)) != values_.end();
    }

    double get_double(std::string_view key, double default_value = 0.0) const {
        const auto it = values_.find(std::string(key));
        return it == values_.end() ? default_value : it->second;
    }

private:
    std::unordered_map<std::string, double> values_;
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
            throw std::invalid_argument(
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
            throw std::invalid_argument(
                "BlockFactory::create: unknown type '" + type_name + "'");
        }
        return it->second(block_name, params);
    }

private:
    std::unordered_map<std::string, BlockCtor> ctors_;
};

} // namespace shared
