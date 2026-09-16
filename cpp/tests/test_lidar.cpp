#include <doctest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "robotsim/config.hpp"
#include "robotsim/geometry.hpp"
#include "robotsim/rng.hpp"
#include "robotsim/world.hpp"

using robotsim::AABB;
using robotsim::Config;
using robotsim::kPi;
using robotsim::RobotState;
using robotsim::Vec2;
using robotsim::World;

namespace {

constexpr double kTol = 1e-9;

// 20 x 20 m map with a single 2 x 2 m box to the right of (10, 10).
Config test_config(int beams, double range) {
    Config cfg;
    cfg.map_width = 20.0;
    cfg.map_height = 20.0;
    cfg.obstacles = {AABB{Vec2{12.0, 9.0}, Vec2{14.0, 11.0}}};
    cfg.lidar_beams = beams;
    cfg.lidar_range = range;
    return cfg;
}

void place(World& world, double x, double y, double heading, double gx, double gy) {
    RobotState state;
    state.position = Vec2{x, y};
    state.heading = heading;
    world.set_episode(state, Vec2{gx, gy});
}

}  // namespace

TEST_CASE("LiDAR ranges on a hand-computed map") {
    SUBCASE("four beams, heading along +x") {
        World world{test_config(4, 15.0)};
        place(world, 10.0, 10.0, 0.0, 5.0, 5.0);
        const std::vector<double> r = world.lidar();
        REQUIRE(r.size() == 4u);
        CHECK(r[0] == doctest::Approx(2.0).epsilon(kTol));   // +x: box face at x = 12
        CHECK(r[1] == doctest::Approx(10.0).epsilon(kTol));  // +y: top wall
        CHECK(r[2] == doctest::Approx(10.0).epsilon(kTol));  // -x: left wall
        CHECK(r[3] == doctest::Approx(10.0).epsilon(kTol));  // -y: bottom wall
    }

    SUBCASE("beams rotate with the heading") {
        World world{test_config(4, 15.0)};
        place(world, 10.0, 10.0, kPi / 2.0, 5.0, 5.0);
        const std::vector<double> r = world.lidar();
        CHECK(r[0] == doctest::Approx(10.0).epsilon(kTol));  // now facing +y
        CHECK(r[3] == doctest::Approx(2.0).epsilon(kTol));   // heading + 3*pi/2 faces +x
    }

    SUBCASE("readings are clamped to the sensor range") {
        World world{test_config(4, 1.5)};
        place(world, 10.0, 10.0, 0.0, 5.0, 5.0);
        const std::vector<double> r = world.lidar();
        for (const double range : r) {
            CHECK(range == 1.5);
        }
    }

    SUBCASE("diagonal beam") {
        // The 45 degree beam enters the box through the middle of its left face
        // at (12, 12). Aiming exactly at a corner is avoided on purpose: sin/cos
        // results may differ by one unit in the last place between math
        // libraries, which can turn a corner hit into a miss.
        Config cfg = test_config(8, 15.0);
        cfg.obstacles = {AABB{Vec2{12.0, 11.5}, Vec2{13.0, 13.0}}};
        World world{cfg};
        place(world, 10.0, 10.0, 0.0, 5.0, 5.0);
        const std::vector<double> r = world.lidar();
        CHECK(r[1] == doctest::Approx(std::sqrt(8.0)).epsilon(kTol));  // 45 degrees
    }

    SUBCASE("beams starting inside an obstacle read zero") {
        World world{test_config(4, 15.0)};
        place(world, 13.0, 10.0, 0.0, 5.0, 5.0);
        for (const double range : world.lidar()) {
            CHECK(range == 0.0);
        }
    }
}

TEST_CASE("observation layout and goal encoding") {
    World world{test_config(4, 15.0)};
    const double diagonal = std::hypot(20.0, 20.0);
    REQUIRE(world.observation_size() == 8);

    SUBCASE("LiDAR part is normalised by the range") {
        place(world, 10.0, 10.0, 0.0, 13.0, 10.0);
        const std::vector<float> obs = world.observation();
        CHECK(obs[0] == doctest::Approx(2.0 / 15.0).epsilon(1e-6));
        CHECK(obs[1] == doctest::Approx(10.0 / 15.0).epsilon(1e-6));
    }

    SUBCASE("goal straight ahead") {
        place(world, 10.0, 10.0, 0.0, 13.0, 10.0);
        const std::vector<float> obs = world.observation();
        CHECK(obs[4] == doctest::Approx(3.0 / diagonal).epsilon(1e-6));
        CHECK(obs[5] == doctest::Approx(0.0).epsilon(1e-6));
        CHECK(obs[6] == doctest::Approx(1.0).epsilon(1e-6));
    }

    SUBCASE("goal to the left") {
        place(world, 10.0, 10.0, 0.0, 10.0, 15.0);
        const std::vector<float> obs = world.observation();
        CHECK(obs[5] == doctest::Approx(1.0).epsilon(1e-6));
        CHECK(std::abs(obs[6]) < 1e-6f);
    }

    SUBCASE("goal to the right after turning") {
        place(world, 10.0, 10.0, kPi / 2.0, 15.0, 10.0);
        const std::vector<float> obs = world.observation();
        CHECK(obs[5] == doctest::Approx(-1.0).epsilon(1e-6));
        CHECK(std::abs(obs[6]) < 1e-6f);
    }

    SUBCASE("goal at the robot position") {
        place(world, 10.0, 10.0, 0.0, 10.0, 10.0);
        const std::vector<float> obs = world.observation();
        CHECK(obs[4] == 0.0f);
        CHECK(obs[5] == 0.0f);
        CHECK(obs[6] == 1.0f);
    }

    SUBCASE("last applied velocity") {
        place(world, 10.0, 10.0, kPi / 2.0, 5.0, 5.0);
        CHECK(world.observation()[7] == 0.0f);
        world.step(0.5, 0.0);
        CHECK(world.observation()[7] == doctest::Approx(0.5).epsilon(1e-6));
    }
}

TEST_CASE("observations stay in bounds during random episodes") {
    World world{Config{}};
    REQUIRE(world.observation_size() == 40);
    world.reset(11);
    robotsim::Rng actions(12);

    const auto size = static_cast<std::size_t>(world.observation_size());
    const std::size_t beams = size - 4;
    std::vector<float> obs(size);

    int out_of_bounds = 0;
    int non_finite = 0;
    for (int i = 0; i < 20000; ++i) {
        world.step(actions.uniform01(), actions.uniform(-1.5, 1.5));
        world.write_observation(obs.data());

        for (std::size_t k = 0; k < size; ++k) {
            if (!std::isfinite(obs[k])) {
                ++non_finite;
            }
        }
        for (std::size_t k = 0; k < beams; ++k) {
            if (obs[k] < 0.0f || obs[k] > 1.0f) {
                ++out_of_bounds;
            }
        }
        const float dist = obs[beams];
        const float sin_b = obs[beams + 1];
        const float cos_b = obs[beams + 2];
        const float v = obs[beams + 3];
        if (dist < 0.0f || dist > 1.0f || sin_b < -1.0f || sin_b > 1.0f || cos_b < -1.0f ||
            cos_b > 1.0f || v < 0.0f || v > 1.0f) {
            ++out_of_bounds;
        }

        if (world.done()) {
            world.reset();
        }
    }
    CHECK(non_finite == 0);
    CHECK(out_of_bounds == 0);
}