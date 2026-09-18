#include "robotsim/platform.hpp"

#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdlib>
#include <memory>
#endif

namespace robotsim {

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
            // The last core of the highest class: logical processor 0 usually
            // handles more interrupts than the others.
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

}  // namespace robotsim