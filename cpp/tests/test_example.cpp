#include <doctest.h>

#include "robotsim/example.hpp"

TEST_CASE("add returns the sum of two integers") {
    CHECK(robotsim::add(2, 3) == 5);
    CHECK(robotsim::add(-4, 4) == 0);
}