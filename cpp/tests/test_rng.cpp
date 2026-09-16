#include <doctest.h>

#include <array>
#include <cstdint>

#include "robotsim/rng.hpp"

using robotsim::Rng;

// Reference values were cross-checked against an independent transcription of
// the public-domain C reference implementations of SplitMix64 and xoshiro256**.

TEST_CASE("splitmix64 matches the reference implementation") {
    std::uint64_t state = 0;
    CHECK(robotsim::splitmix64(state) == 0xE220A8397B1DCDAFULL);
}

TEST_CASE("Rng matches reference sequences") {
    SUBCASE("seed 0") {
        Rng rng(0);
        CHECK(rng.next_u64() == 0x99EC5F36CB75F2B4ULL);
        CHECK(rng.next_u64() == 0xBF6E1F784956452AULL);
        CHECK(rng.next_u64() == 0x1A5F849D4933E6E0ULL);
        CHECK(rng.next_u64() == 0x6AA594F1262D2D2CULL);
    }
    SUBCASE("seed 42") {
        Rng rng(42);
        CHECK(rng.next_u64() == 0x15780B2E0C2EC716ULL);
        CHECK(rng.next_u64() == 0x6104D9866D113A7EULL);
        CHECK(rng.next_u64() == 0xAE17533239E499A1ULL);
        CHECK(rng.next_u64() == 0xECB8AD4703B360A1ULL);
        // uniform01 is an exact conversion, so doubles can be compared exactly.
        CHECK(rng.uniform01() == 0x1.fbcdb8ffc5d8bp-1);
        CHECK(rng.uniform01() == 0x1.8a1b4a6202f2ap-1);
    }
}

TEST_CASE("Rng is usable at compile time") {
    static constexpr std::uint64_t first = Rng(0).next_u64();
    static_assert(first == 0x99EC5F36CB75F2B4ULL);
    // The largest possible uniform01 value is strictly below 1.
    static_assert(static_cast<double>(UINT64_MAX >> 11) * 0x1.0p-53 < 1.0);
    CHECK(first == 0x99EC5F36CB75F2B4ULL);
}

TEST_CASE("same seed gives the same sequence, reseed restarts it") {
    Rng a(123456789);
    Rng b(123456789);
    int mismatches = 0;
    for (int i = 0; i < 10000; ++i) {
        if (a.next_u64() != b.next_u64()) {
            ++mismatches;
        }
    }
    CHECK(mismatches == 0);

    Rng c(7);
    std::array<std::uint64_t, 16> first{};
    for (auto& value : first) {
        value = c.next_u64();
    }
    c.reseed(7);
    for (const auto value : first) {
        CHECK(c.next_u64() == value);
    }
}

TEST_CASE("neighbouring seeds give different sequences") {
    Rng a(1000);
    Rng b(1001);
    int equal = 0;
    for (int i = 0; i < 1000; ++i) {
        if (a.next_u64() == b.next_u64()) {
            ++equal;
        }
    }
    CHECK(equal == 0);
}

TEST_CASE("uniform01 and uniform stay in range") {
    // Track min/max instead of asserting inside the loop: one million doctest
    // assertions would dominate the test run time.
    Rng rng(2024);
    const int n = 1000000;
    double sum = 0.0;
    double lo = 1.0;
    double hi = 0.0;
    for (int i = 0; i < n; ++i) {
        const double u = rng.uniform01();
        sum += u;
        lo = u < lo ? u : lo;
        hi = u > hi ? u : hi;
    }
    CHECK(lo >= 0.0);
    CHECK(hi < 1.0);
    CHECK(sum / n == doctest::Approx(0.5).epsilon(0.005));

    double vlo = 7.5;
    double vhi = -2.5;
    for (int i = 0; i < 100000; ++i) {
        const double v = rng.uniform(-2.5, 7.5);
        vlo = v < vlo ? v : vlo;
        vhi = v > vhi ? v : vhi;
    }
    CHECK(vlo >= -2.5);
    CHECK(vhi < 7.5);
}

TEST_CASE("uniform_int is in range and roughly uniform") {
    Rng rng(99);

    std::uint64_t max_single = 0;
    for (int i = 0; i < 1000; ++i) {
        const std::uint64_t k = rng.uniform_int(1);
        max_single = k > max_single ? k : max_single;
    }
    CHECK(max_single == 0);

    std::array<int, 10> counts{};
    const int n = 100000;
    int out_of_range = 0;
    for (int i = 0; i < n; ++i) {
        const std::uint64_t k = rng.uniform_int(counts.size());
        if (k < counts.size()) {
            ++counts[k];
        } else {
            ++out_of_range;
        }
    }
    CHECK(out_of_range == 0);
    // Expected 10000 per bucket; standard deviation is about 95.
    for (const int count : counts) {
        CHECK(count > 9500);
        CHECK(count < 10500);
    }
}