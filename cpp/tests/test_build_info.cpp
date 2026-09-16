#include <doctest.h>

#include "robotsim/build_info.hpp"

TEST_CASE("build_info describes the build") {
    const std::string info = robotsim::build_info();
    CHECK(info.rfind("robotsim", 0) == 0);
    CHECK(info.find("64-bit") != std::string::npos);
    CHECK(__cplusplus >= 201703L);
}