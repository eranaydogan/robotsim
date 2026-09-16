#include <doctest.h>

#include <cstdint>
#include <stdexcept>

#include "robotsim/config.hpp"
#include "robotsim/geometry.hpp"
#include "robotsim/world.hpp"

using robotsim::AABB;
using robotsim::Config;
using robotsim::kPi;
using robotsim::Vec2;
using robotsim::World;

namespace {

bool free_at(const World& world, double x, double y, double radius) {
    return world.is_free(Vec2{x, y}, radius);
}

bool same_episode(const World& a, const World& b) {
    return a.robot().position.x == b.robot().position.x &&
           a.robot().position.y == b.robot().position.y &&
           a.robot().heading == b.robot().heading && a.goal().x == b.goal().x &&
           a.goal().y == b.goal().y;
}

}  // namespace

TEST_CASE("Config validation") {
    CHECK_NOTHROW(robotsim::validate(Config{}));

    SUBCASE("non-positive robot radius") {
        Config cfg;
        cfg.robot_radius = 0.0;
        CHECK_THROWS_AS(robotsim::validate(cfg), std::invalid_argument);
    }
    SUBCASE("non-positive dt") {
        Config cfg;
        cfg.dt = -0.1;
        CHECK_THROWS_AS(robotsim::validate(cfg), std::invalid_argument);
    }
    SUBCASE("zero lidar beams") {
        Config cfg;
        cfg.lidar_beams = 0;
        CHECK_THROWS_AS(robotsim::validate(cfg), std::invalid_argument);
    }
    SUBCASE("degenerate obstacle") {
        Config cfg;
        cfg.obstacles.push_back(AABB{Vec2{1.0, 1.0}, Vec2{1.0, 2.0}});
        CHECK_THROWS_AS(robotsim::validate(cfg), std::invalid_argument);
    }
    SUBCASE("map too small for the robot") {
        Config cfg;
        cfg.map_width = 0.5;
        CHECK_THROWS_AS(World{cfg}, std::invalid_argument);
    }
}

TEST_CASE("default obstacles lie inside the map") {
    const Config cfg;
    CHECK(cfg.obstacles.size() == 10u);
    for (const AABB& box : cfg.obstacles) {
        CHECK(box.lo.x >= 0.0);
        CHECK(box.lo.y >= 0.0);
        CHECK(box.hi.x <= cfg.map_width);
        CHECK(box.hi.y <= cfg.map_height);
    }
}

TEST_CASE("World adds four boundary walls") {
    const World world{Config{}};
    CHECK(world.obstacles().size() == world.config().obstacles.size() + 4u);
}

TEST_CASE("is_free") {
    const World world{Config{}};
    const double r = 0.2;

    // Decimal inputs such as 2.7 - 2.5 are not exactly 0.2 in binary floating
    // point, so exact tangency is only tested with representable values.
    CHECK(free_at(world, 5.0, 1.0, r));            // open floor
    CHECK_FALSE(free_at(world, 2.0, 2.25, r));     // inside a shelf
    CHECK_FALSE(free_at(world, 2.0, 2.69, r));     // overlapping a shelf edge
    CHECK(free_at(world, 2.0, 2.71, r));           // just clear of the shelf
    CHECK_FALSE(free_at(world, 0.19, 1.0, r));     // overlapping the left wall
    CHECK_FALSE(free_at(world, 0.25, 1.0, 0.25));  // exactly touching the wall is blocked
    CHECK(free_at(world, 0.21, 1.0, r));           // just clear of the left wall
    CHECK_FALSE(free_at(world, -1.0, 5.0, r));     // outside the map
    CHECK_FALSE(free_at(world, 5.0, 10.5, r));     // outside the map
}

TEST_CASE("reset produces valid episodes") {
    World world{Config{}};
    const Config& cfg = world.config();
    const double spawn_radius = cfg.robot_radius + cfg.spawn_clearance;
    const double min_dist = cfg.min_start_goal_distance;

    int invalid_start = 0;
    int invalid_goal = 0;
    int too_close = 0;
    int bad_heading = 0;
    int bad_state = 0;

    for (std::uint64_t seed = 0; seed < 10000; ++seed) {
        world.reset(seed);
        const auto& robot = world.robot();

        if (!world.is_free(robot.position, spawn_radius)) {
            ++invalid_start;
        }
        if (!world.is_free(world.goal(), spawn_radius)) {
            ++invalid_goal;
        }
        if (robotsim::length(world.goal() - robot.position) < min_dist) {
            ++too_close;
        }
        if (!(robot.heading >= -kPi && robot.heading < kPi)) {
            ++bad_heading;
        }
        if (robot.v != 0.0 || robot.omega != 0.0 || world.step_count() != 0) {
            ++bad_state;
        }
    }

    CHECK(invalid_start == 0);
    CHECK(invalid_goal == 0);
    CHECK(too_close == 0);
    CHECK(bad_heading == 0);
    CHECK(bad_state == 0);
}

TEST_CASE("reset is deterministic") {
    World a{Config{}};
    World b{Config{}};

    SUBCASE("same seed gives the same episode") {
        a.reset(1234);
        b.reset(1234);
        CHECK(same_episode(a, b));
    }
    SUBCASE("different seeds give different episodes") {
        a.reset(1234);
        b.reset(1235);
        CHECK_FALSE(same_episode(a, b));
    }
    SUBCASE("unseeded reset continues the stream reproducibly") {
        a.reset(77);
        a.reset();
        b.reset(77);
        b.reset();
        CHECK(same_episode(a, b));

        World c{Config{}};
        c.reset(77);
        CHECK_FALSE(same_episode(a, c));
    }
}

TEST_CASE("impossible spawn constraints throw") {
    Config cfg;
    cfg.min_start_goal_distance = 100.0;  // larger than the map diagonal
    cfg.max_spawn_attempts = 100;
    CHECK_THROWS_AS(World{cfg}, std::runtime_error);
}