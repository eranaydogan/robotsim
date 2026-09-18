// Micro-benchmark for the single-environment C++ core.
//
// Usage: robotsim_bench [--quick] [--reps N] [--pin]
//   --quick   run 10x fewer iterations (smoke test for CI)
//   --reps N  number of timed repetitions per benchmark (default 5)
//   --pin     pin the benchmark thread to one performance core (Windows only).
//             On hybrid CPUs the scheduler may otherwise move the thread
//             between performance and efficiency cores during a run.
//
// Each benchmark runs a fixed, seeded workload. The median rate over all
// repetitions is reported together with the minimum and maximum. A checksum of
// the simulated data is printed so that runs of the same binary can be checked
// for doing identical work (see D-007 for determinism across compilers).

#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "robotsim/build_info.hpp"
#include "robotsim/config.hpp"
#include "robotsim/rng.hpp"
#include "robotsim/world.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct Stats {
    double median = 0.0;
    double min = 0.0;
    double max = 0.0;
};

// Runs `body` once untimed as a warm-up, then `reps` timed times.
// Returns iterations per second statistics.
Stats measure(int reps, std::int64_t iterations, const std::function<void()>& body) {
    body();
    std::vector<double> rates;
    rates.reserve(static_cast<std::size_t>(reps));
    for (int r = 0; r < reps; ++r) {
        const auto start = Clock::now();
        body();
        const std::chrono::duration<double> elapsed = Clock::now() - start;
        rates.push_back(static_cast<double>(iterations) / elapsed.count());
    }
    std::sort(rates.begin(), rates.end());
    Stats stats;
    stats.min = rates.front();
    stats.max = rates.back();
    const std::size_t mid = rates.size() / 2;
    stats.median = (rates.size() % 2 == 1) ? rates[mid] : 0.5 * (rates[mid - 1] + rates[mid]);
    return stats;
}

// Pins the calling thread to a single logical processor of the highest
// efficiency class (a performance core on hybrid CPUs). The last such core is
// chosen to stay away from logical processor 0, which usually handles more
// interrupts. Returns a description, or an empty string if pinning failed.
std::string pin_to_performance_core() {
#ifdef _WIN32
    DWORD length = 0;
    GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &length);
    if (length == 0) {
        return {};
    }
    // malloc returns memory aligned for any fundamental type, which the
    // variable-length records require.
    std::unique_ptr<void, decltype(&std::free)> buffer(std::malloc(length), &std::free);
    if (!buffer) {
        return {};
    }
    auto* const base = static_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(buffer.get());
    if (!GetLogicalProcessorInformationEx(RelationProcessorCore, base, &length)) {
        return {};
    }

    int best_cpu = -1;
    int best_class = -1;
    for (DWORD offset = 0; offset < length;) {
        const auto* entry = static_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(
            static_cast<void*>(static_cast<char*>(buffer.get()) + offset));
        const PROCESSOR_RELATIONSHIP& core = entry->Processor;
        const GROUP_AFFINITY& group = core.GroupMask[0];
        if (group.Group == 0 && group.Mask != 0) {
            int first_cpu = 0;
            while (((group.Mask >> first_cpu) & 1) == 0) {
                ++first_cpu;
            }
            const int efficiency = static_cast<int>(core.EfficiencyClass);
            if (efficiency >= best_class) {
                best_class = efficiency;
                best_cpu = first_cpu;
            }
        }
        offset += entry->Size;
    }
    if (best_cpu < 0) {
        return {};
    }
    const DWORD_PTR mask = static_cast<DWORD_PTR>(1) << best_cpu;
    if (SetThreadAffinityMask(GetCurrentThread(), mask) == 0) {
        return {};
    }
    return "logical processor " + std::to_string(best_cpu) + " (efficiency class " +
           std::to_string(best_class) + ")";
#else
    return {};
#endif
}

void print_row(const char* name, std::int64_t iterations, const Stats& s) {
    std::printf("| %-26s | %11" PRId64 " | %12.0f | %12.0f | %12.0f |\n", name, iterations,
                s.median, s.min, s.max);
}

}  // namespace

int main(int argc, char** argv) {
    bool quick = false;
    bool pin = false;
    int reps = 5;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--quick") == 0) {
            quick = true;
        } else if (std::strcmp(argv[i], "--pin") == 0) {
            pin = true;
        } else if (std::strcmp(argv[i], "--reps") == 0 && i + 1 < argc) {
            reps = std::max(1, std::atoi(argv[++i]));
        } else {
            std::fprintf(stderr, "usage: robotsim_bench [--quick] [--reps N] [--pin]\n");
            return 2;
        }
    }
    std::string pinned = "no";
    if (pin) {
        const std::string where = pin_to_performance_core();
        pinned = where.empty() ? "requested but not available" : where;
    }
    const std::int64_t scale = quick ? 10 : 1;

    const robotsim::Config cfg;
    const std::int64_t n_reset = 1000000 / scale;
    const std::int64_t n_step = 5000000 / scale;
    const std::int64_t n_scan = 1000000 / scale;
    const std::int64_t n_step_obs = 1000000 / scale;

    std::printf("# robotsim C++ core benchmark\n\n");
    std::printf("- Build: %s\n", robotsim::build_info().c_str());
    std::printf("- Hardware threads: %u\n", std::thread::hardware_concurrency());
    std::printf("- Configuration: default (%d LiDAR beams, %zu obstacles + 4 walls)\n",
                cfg.lidar_beams, cfg.obstacles.size());
    std::printf("- Repetitions: %d timed runs after one warm-up, single thread%s\n", reps,
                quick ? ", quick mode" : "");
    std::printf("- Thread pinned: %s\n\n", pinned.c_str());

    std::printf("| %-26s | %11s | %12s | %12s | %12s |\n", "Benchmark", "Iterations",
                "Median /s", "Min /s", "Max /s");
    std::printf("|%s|%s|%s|%s|%s|\n", "----------------------------", "-------------",
                "--------------", "--------------", "--------------");

    // reset(): rejection sampling of start and goal.
    {
        robotsim::World world{cfg};
        const Stats s = measure(reps, n_reset, [&] {
            world.reset(1);
            for (std::int64_t i = 0; i < n_reset; ++i) {
                world.reset();
            }
        });
        print_row("reset", n_reset, s);
    }

    // step() without observation.
    {
        robotsim::World world{cfg};
        const Stats s = measure(reps, n_step, [&] {
            world.reset(1);
            robotsim::Rng actions(5);
            for (std::int64_t i = 0; i < n_step; ++i) {
                world.step(actions.uniform01(), actions.uniform(-1.5, 1.5));
                if (world.done()) {
                    world.reset();
                }
            }
        });
        print_row("step", n_step, s);
    }

    // scan_lidar() at a fixed pose.
    {
        robotsim::World world{cfg};
        world.reset(1);
        std::vector<double> ranges(static_cast<std::size_t>(cfg.lidar_beams));
        const Stats s = measure(reps, n_scan, [&] {
            for (std::int64_t i = 0; i < n_scan; ++i) {
                world.scan_lidar(ranges.data());
            }
        });
        print_row("scan_lidar", n_scan, s);
    }

    // step() + write_observation(): the per-step cost seen by an RL agent.
    double checksum = 0.0;
    std::int64_t episodes = 0;
    {
        robotsim::World world{cfg};
        std::vector<float> obs(static_cast<std::size_t>(world.observation_size()));
        const Stats s = measure(reps, n_step_obs, [&] {
            world.reset(1);
            robotsim::Rng actions(5);
            double sum = 0.0;
            std::int64_t ended = 0;
            for (std::int64_t i = 0; i < n_step_obs; ++i) {
                const robotsim::StepResult result =
                    world.step(actions.uniform01(), actions.uniform(-1.5, 1.5));
                world.write_observation(obs.data());
                sum += result.reward;
                for (const float value : obs) {
                    sum += static_cast<double>(value);
                }
                if (world.done()) {
                    ++ended;
                    world.reset();
                }
            }
            checksum = sum;
            episodes = ended;
        });
        print_row("step + write_observation", n_step_obs, s);
    }

    std::printf("\n- Episodes in step + observation workload: %" PRId64 "\n", episodes);
    // The hexadecimal form shows every bit, so equal values mean identical results.
    std::printf("- Checksum of rewards and observations: %.6f (%a)\n", checksum, checksum);
    return 0;
}