#pragma once

#include <cstdint>

namespace robotsim {

// ---------------------------------------------------------------------------
// Portable random number generation
//
// Standard library distributions (std::uniform_real_distribution, ...) are
// not used because their output is implementation-defined and may differ
// between compilers and standard library versions. Everything here is built
// from exact 64-bit integer arithmetic, so the same seed produces the same
// sequence on every platform.
// ---------------------------------------------------------------------------

// SplitMix64 step: advances state and returns the next output.
// Used to expand a single 64-bit seed into the xoshiro state.
constexpr std::uint64_t splitmix64(std::uint64_t& state) {
    state += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

// xoshiro256** 1.0 (Blackman & Vigna).
class Rng {
public:
    explicit constexpr Rng(std::uint64_t seed = 0) { reseed(seed); }

    constexpr void reseed(std::uint64_t seed) {
        std::uint64_t sm = seed;
        for (auto& word : s_) {
            word = splitmix64(sm);
        }
    }

    constexpr std::uint64_t next_u64() {
        const std::uint64_t result = rotl(s_[1] * 5, 7) * 9;
        const std::uint64_t t = s_[1] << 17;
        s_[2] ^= s_[0];
        s_[3] ^= s_[1];
        s_[1] ^= s_[2];
        s_[0] ^= s_[3];
        s_[2] ^= t;
        s_[3] = rotl(s_[3], 45);
        return result;
    }

    // Uniform double in [0, 1) with 53 bits of resolution. The conversion is
    // exact, so the result is bit-identical on all IEEE 754 platforms.
    constexpr double uniform01() {
        return static_cast<double>(next_u64() >> 11) * 0x1.0p-53;
    }

    // Uniform double between lo and hi. Nominally [lo, hi); floating-point
    // rounding can very rarely produce exactly hi.
    constexpr double uniform(double lo, double hi) { return lo + (hi - lo) * uniform01(); }

    // Uniform integer in [0, n) without modulo bias. Requires n > 0.
    constexpr std::uint64_t uniform_int(std::uint64_t n) {
        // Values below the threshold would make some residues more likely.
        const std::uint64_t threshold = (0 - n) % n;
        std::uint64_t x = next_u64();
        while (x < threshold) {
            x = next_u64();
        }
        return x % n;
    }

private:
    static constexpr std::uint64_t rotl(std::uint64_t x, int k) {
        return (x << k) | (x >> (64 - k));
    }

    std::uint64_t s_[4]{};
};

}  // namespace robotsim