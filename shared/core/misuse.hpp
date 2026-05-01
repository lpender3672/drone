#pragma once

#include <cstdio>
#include <cstdlib>
#include <string>

#if __cpp_exceptions || defined(__EXCEPTIONS)
  #include <stdexcept>
#endif

namespace shared {
namespace detail {

// Misconfiguration helpers used by Tier 2 graph code (Graph, BlockFactory,
// IPort dispatch). All call sites are startup-time misuse — duplicate
// names, missing ports, type mismatches, cycles. The hot loop never
// invokes these.
//
// On host builds (sim, tests) where exceptions are enabled, they throw
// `std::invalid_argument` / `std::runtime_error` so EXPECT_THROW works
// and so user code can catch and present a friendly error.
//
// On embedded targets where exceptions are disabled (Teensy 4.1's memory
// map can't fit ARM unwind tables — .flashmem at 0x60000000 is beyond
// PREL31 range from .text), the throw becomes a fail-fast log + abort.
// A graph misconfig is always a programmer error, so loud abort is the
// right behaviour: better than silent wrong wiring.

[[noreturn]] inline void invalid_argument(const std::string& msg) {
#if __cpp_exceptions || defined(__EXCEPTIONS)
    throw std::invalid_argument(msg);
#else
    std::fputs("misuse (invalid_argument): ", stderr);
    std::fputs(msg.c_str(), stderr);
    std::fputc('\n', stderr);
    std::abort();
#endif
}

[[noreturn]] inline void runtime_error(const std::string& msg) {
#if __cpp_exceptions || defined(__EXCEPTIONS)
    throw std::runtime_error(msg);
#else
    std::fputs("misuse (runtime_error): ", stderr);
    std::fputs(msg.c_str(), stderr);
    std::fputc('\n', stderr);
    std::abort();
#endif
}

} // namespace detail
} // namespace shared
