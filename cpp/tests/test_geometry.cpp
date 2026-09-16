#include <doctest.h>

#include <cmath>
#include <optional>

#include "robotsim/geometry.hpp"

using robotsim::AABB;
using robotsim::kPi;
using robotsim::Vec2;

// Note: braced initializers such as Vec2{1.0, 2.0} must not appear directly
// inside CHECK(...), because the preprocessor treats their commas as macro
// argument separators. Values are assigned to variables first.

namespace {

constexpr double kTol = 1e-12;

std::optional<double> shoot(double ox, double oy, double dx, double dy, const AABB& box) {
    return robotsim::ray_aabb(Vec2{ox, oy}, Vec2{dx, dy}, box);
}

bool circle_hits(double cx, double cy, double radius, const AABB& box) {
    return robotsim::circle_intersects_aabb(Vec2{cx, cy}, radius, box);
}

}  // namespace

TEST_CASE("Vec2 arithmetic") {
    const Vec2 a{1.0, 2.0};
    const Vec2 b{3.0, -4.0};

    const Vec2 sum = a + b;
    CHECK(sum.x == 4.0);
    CHECK(sum.y == -2.0);

    const Vec2 diff = a - b;
    CHECK(diff.x == -2.0);
    CHECK(diff.y == 6.0);

    const Vec2 scaled_right = a * 2.0;
    const Vec2 scaled_left = 2.0 * a;
    CHECK(scaled_right.x == 2.0);
    CHECK(scaled_right.y == 4.0);
    CHECK(scaled_left.x == 2.0);
    CHECK(scaled_left.y == 4.0);

    CHECK(robotsim::dot(a, b) == -5.0);
    CHECK(robotsim::length(b) == doctest::Approx(5.0).epsilon(kTol));
}

TEST_CASE("wrap_angle maps angles to [-pi, pi)") {
    SUBCASE("known values") {
        CHECK(robotsim::wrap_angle(0.0) == doctest::Approx(0.0).epsilon(kTol));
        CHECK(robotsim::wrap_angle(kPi) == -kPi);
        CHECK(robotsim::wrap_angle(-kPi) == -kPi);
        CHECK(robotsim::wrap_angle(kPi + 0.1) == doctest::Approx(-kPi + 0.1).epsilon(kTol));
        CHECK(robotsim::wrap_angle(-kPi - 0.1) == doctest::Approx(kPi - 0.1).epsilon(kTol));
    }

    SUBCASE("range and equivalence over many turns") {
        for (int i = -2000; i <= 2000; ++i) {
            const double angle = static_cast<double>(i) * 0.0503;
            const double wrapped = robotsim::wrap_angle(angle);
            CHECK(wrapped >= -kPi);
            CHECK(wrapped < kPi);
            CHECK(std::cos(wrapped) == doctest::Approx(std::cos(angle)).epsilon(1e-9));
            CHECK(std::sin(wrapped) == doctest::Approx(std::sin(angle)).epsilon(1e-9));
        }
    }
}

TEST_CASE("closest_point clamps to the box") {
    const AABB box{Vec2{0.0, 0.0}, Vec2{1.0, 1.0}};

    const Vec2 inside = robotsim::closest_point(box, Vec2{0.25, 0.75});
    CHECK(inside.x == 0.25);
    CHECK(inside.y == 0.75);

    const Vec2 left = robotsim::closest_point(box, Vec2{-2.0, 0.5});
    CHECK(left.x == 0.0);
    CHECK(left.y == 0.5);

    const Vec2 corner = robotsim::closest_point(box, Vec2{3.0, 4.0});
    CHECK(corner.x == 1.0);
    CHECK(corner.y == 1.0);
}

TEST_CASE("circle_intersects_aabb") {
    const AABB box{Vec2{0.0, 0.0}, Vec2{1.0, 1.0}};

    SUBCASE("center inside the box") {
        CHECK(circle_hits(0.5, 0.5, 0.1, box));
    }
    SUBCASE("overlapping an edge") {
        CHECK(circle_hits(1.2, 0.5, 0.5, box));
    }
    SUBCASE("exactly tangent to an edge counts as a collision") {
        CHECK(circle_hits(1.5, 0.5, 0.5, box));
    }
    SUBCASE("1e-4 away from an edge") {
        CHECK_FALSE(circle_hits(1.5001, 0.5, 0.5, box));
    }
    SUBCASE("exactly tangent to a corner") {
        // Offset to corner (1, 1) is (0.75, 1.0), length exactly 1.25.
        CHECK(circle_hits(1.75, 2.0, 1.25, box));
        CHECK_FALSE(circle_hits(1.75, 2.0, 1.2499, box));
    }
}

TEST_CASE("ray_aabb") {
    const AABB box{Vec2{2.0, -1.0}, Vec2{4.0, 1.0}};

    SUBCASE("axis-aligned hit") {
        const auto hit = shoot(0.0, 0.0, 1.0, 0.0, box);
        REQUIRE(hit.has_value());
        CHECK(*hit == doctest::Approx(2.0).epsilon(kTol));
    }
    SUBCASE("box behind the ray") {
        CHECK_FALSE(shoot(0.0, 0.0, -1.0, 0.0, box).has_value());
    }
    SUBCASE("parallel ray outside the slab") {
        CHECK_FALSE(shoot(0.0, 0.0, 0.0, 1.0, box).has_value());
    }
    SUBCASE("parallel ray inside the slab") {
        const auto hit = shoot(3.0, -5.0, 0.0, 1.0, box);
        REQUIRE(hit.has_value());
        CHECK(*hit == doctest::Approx(4.0).epsilon(kTol));
    }
    SUBCASE("origin inside the box") {
        const auto hit = shoot(3.0, 0.0, 1.0, 0.0, box);
        REQUIRE(hit.has_value());
        CHECK(*hit == 0.0);
    }
    SUBCASE("diagonal miss") {
        const double s = 1.0 / std::sqrt(2.0);
        CHECK_FALSE(shoot(0.0, 0.0, s, s, box).has_value());
    }

    const AABB unit{Vec2{1.0, 1.0}, Vec2{2.0, 2.0}};

    SUBCASE("diagonal hit") {
        const double s = 1.0 / std::sqrt(2.0);
        const auto hit = shoot(0.0, 0.0, s, s, unit);
        REQUIRE(hit.has_value());
        CHECK(*hit == doctest::Approx(std::sqrt(2.0)).epsilon(1e-9));
    }
    SUBCASE("ray grazing only a corner counts as a hit") {
        // From (2, 0) toward (1, 1): touches the box only at corner (1, 1).
        const double s = 1.0 / std::sqrt(2.0);
        const auto hit = shoot(2.0, 0.0, -s, s, unit);
        REQUIRE(hit.has_value());
        CHECK(*hit == doctest::Approx(std::sqrt(2.0)).epsilon(1e-9));
    }
}