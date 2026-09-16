#pragma once

#include <string>

namespace robotsim {

// Returns a short description of the compiler and build configuration.
// Used to label benchmark results.
std::string build_info();

}  // namespace robotsim