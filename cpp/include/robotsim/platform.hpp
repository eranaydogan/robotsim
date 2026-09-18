#pragma once

#include <string>

namespace robotsim {

// Pins the calling thread to a single logical processor of the highest
// efficiency class reported by the operating system (a performance core on
// hybrid CPUs), so that benchmark timings are not disturbed by the scheduler
// moving the thread between core types.
//
// Returns a description of the chosen processor, or an empty string if pinning
// is not available on this platform or failed.
std::string pin_to_performance_core();

}  // namespace robotsim