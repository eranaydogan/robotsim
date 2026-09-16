#include "robotsim/build_info.hpp"

namespace robotsim {

std::string build_info() {
    std::string info = "robotsim";

#if defined(_MSC_FULL_VER)
    info += " | MSVC " + std::to_string(_MSC_FULL_VER);
#elif defined(__clang__)
    info += " | Clang " + std::to_string(__clang_major__) + "." + std::to_string(__clang_minor__);
#elif defined(__GNUC__)
    info += " | GCC " + std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__);
#else
    info += " | unknown compiler";
#endif

    info += " | C++ " + std::to_string(__cplusplus);
    info += " | " + std::to_string(sizeof(void*) * 8) + "-bit";

#if defined(NDEBUG)
    info += " | Release";
#else
    info += " | Debug";
#endif

    return info;
}

}  // namespace robotsim