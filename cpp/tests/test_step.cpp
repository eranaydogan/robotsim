#include <doctest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "robotsim/config.hpp"
#include "robotsim/geometry.hpp"
#include "robotsim/rng.hpp"
#include "robotsim/world.hpp"

using robotsim::Config;
using robotsim::kPi;
using robotsim::RobotState;
using robotsim::StepResult;
using robotsim::Vec2;
using robotsim::World;

namespace {

constexpr double kTol = 1e-9;

// A 20 x 20 m map without interior obstacles for kinematics tests.
Config open_config() {
    Config cfg;
    cfg.map_width = 20.0;
    cfg.map_height = 20.0;
    cfg.obstacles.clear();
    return cfg;
}

RobotState pose(double x, double y, double heading) {
    RobotState state;
    state.position = Vec2{x, y};
    state.heading = heading;
    return state;
}

void place(World& world, double x, double y, double heading, double gx, double gy) {
    world.set_episode(pose(x, y, heading), Vec2{gx, gy});
}

}  // namespace

TEST_CASE("kinematics match analytic solutions") {
    World world{open_config()};

    SUBCASE("straight line") {
        place(world, 5.0, 5.0, 0.0, 15.0, 15.0);
        for (int i = 0; i < 10; ++i) {
            world.step(1.0, 0.0);
        }
        CHECK(world.robot().position.x == doctest::Approx(6.0).epsilon(kTol));
        CHECK(world.robot().position.y == doctest::Approx(5.0).epsilon(kTol));
        CHECK(world.robot().heading == doctest::Approx(0.0).epsilon(kTol));
    }

    SUBCASE("rotation in place") {
        place(world, 5.0, 5.0, 0.0, 15.0, 15.0);
        for (int i = 0; i < 10; ++i) {
            world.step(0.0, 1.0);
        }
        CHECK(world.robot().position.x == 5.0);
        CHECK(world.robot().position.y == 5.0);
        CHECK(world.robot().heading == doctest::Approx(1.0).epsilon(kTol));
    }

    SUBCASE("constant turn follows a circle") {
        // v = 1 m/s and omega = 1 rad/s give a radius of 1 m. Starting at
        // (10, 10) facing +x, the circle is centred at (10, 11).
        place(world, 10.0, 10.0, 0.0, 1.0, 1.0);
        double max_error = 0.0;
        for (int k = 1; k <= 60; ++k) {
            world.step(1.0, 1.0);
            const double t = k * world.config().dt;
            const double ex = world.robot().position.x - (10.0 + std::sin(t));
            const double ey = world.robot().position.y - (11.0 - std::cos(t));
            max_error = std::max(max_error, std::hypot(ex, ey));
        }
        CHECK(max_error < kTol);
    }
}

TEST_CASE("commands are clamped and validated") {
    World world{open_config()};
    const Config& cfg = world.config();
    place(world, 5.0, 5.0, 0.0, 15.0, 15.0);

    world.step(5.0, 10.0);
    CHECK(world.robot().v == cfg.v_max);
    CHECK(world.robot().omega == cfg.omega_max);

    world.step(-1.0, -10.0);
    CHECK(world.robot().v == 0.0);
    CHECK(world.robot().omega == -cfg.omega_max);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS(world.step(nan, 0.0), std::invalid_argument);
    CHECK_THROWS_AS(world.step(0.0, inf), std::invalid_argument);
}

TEST_CASE("config rejects speeds that allow tunnelling") {
    Config cfg;
    cfg.v_max = 5.0;  // 5 m/s * 0.1 s = 0.5 m > robot radius 0.2 m
    CHECK_THROWS_AS(robotsim::validate(cfg), std::invalid_argument);
}

TEST_CASE("collision ends the episode") {
    World world{open_config()};
    const Config& cfg = world.config();
    // Facing the left wall. After one step x = 0.25 (clear), after two x = 0.15.
    place(world, 0.35, 10.0, kPi, 15.0, 10.0);

    const StepResult first = world.step(1.0, 0.0);
    CHECK_FALSE(first.collision);
    CHECK_FALSE(first.terminated);

    const StepResult second = world.step(1.0, 0.0);
    CHECK(second.collision);
    CHECK(second.terminated);
    CHECK_FALSE(second.truncated);
    CHECK_FALSE(second.is_success);
    CHECK(world.done());
    // Moving 0.1 m away from the goal, step cost and collision penalty.
    CHECK(second.reward ==
          doctest::Approx(-0.1 - cfg.reward_step - cfg.reward_collision).epsilon(kTol));

    CHECK_THROWS_AS(world.step(1.0, 0.0), std::logic_error);
}

TEST_CASE("reaching the goal ends the episode") {
    World world{open_config()};
    const Config& cfg = world.config();
    // Goal 0.45 m ahead: distance 0.35 after one step, 0.25 (< 0.3) after two.
    place(world, 5.0, 5.0, 0.0, 5.45, 5.0);

    const StepResult first = world.step(1.0, 0.0);
    CHECK_FALSE(first.is_success);
    CHECK(first.reward == doctest::Approx(0.1 - cfg.reward_step).epsilon(kTol));

    const StepResult second = world.step(1.0, 0.0);
    CHECK(second.is_success);
    CHECK(second.terminated);
    CHECK_FALSE(second.collision);
    CHECK(second.reward == doctest::Approx(0.1 - cfg.reward_step + cfg.reward_goal).epsilon(kTol));
}

TEST_CASE("collision takes priority over reaching the goal") {
    World world{open_config()};
    // After one step x = 0.18: inside the goal radius but touching the wall.
    place(world, 0.28, 10.0, kPi, 0.3, 10.0);
    const StepResult result = world.step(1.0, 0.0);
    CHECK(result.collision);
    CHECK_FALSE(result.is_success);
}

TEST_CASE("moving away from the goal is penalised") {
    World world{open_config()};
    const Config& cfg = world.config();
    place(world, 5.0, 5.0, 0.0, 2.0, 5.0);
    const StepResult result = world.step(1.0, 0.0);
    CHECK(result.reward == doctest::Approx(-0.1 - cfg.reward_step).epsilon(kTol));
}

TEST_CASE("episode is truncated after max_steps") {
    Config cfg = open_config();
    cfg.max_steps = 5;
    World world{cfg};
    place(world, 5.0, 5.0, 0.0, 15.0, 15.0);

    for (int i = 1; i < 5; ++i) {
        const StepResult result = world.step(0.0, 0.0);
        CHECK_FALSE(result.truncated);
        CHECK_FALSE(world.done());
    }
    const StepResult last = world.step(0.0, 0.0);
    CHECK(last.truncated);
    CHECK_FALSE(last.terminated);
    CHECK(world.done());
    CHECK(world.step_count() == 5);

    world.reset(1);
    CHECK_FALSE(world.done());
    CHECK(world.step_count() == 0);
}

TEST_CASE("simulation is bit-for-bit deterministic") {
    // Two worlds driven by the same seeds and the same random actions, with
    // automatic resets, must produce identical states and rewards.
    World a{Config{}};
    World b{Config{}};
    a.reset(2026);
    b.reset(2026);
    robotsim::Rng actions_a(99);
    robotsim::Rng actions_b(99);

    int mismatches = 0;
    int episodes = 0;
    for (int i = 0; i < 20000; ++i) {
        const StepResult ra = a.step(actions_a.uniform01(), actions_a.uniform(-1.5, 1.5));
        const StepResult rb = b.step(actions_b.uniform01(), actions_b.uniform(-1.5, 1.5));

        const bool same = ra.reward == rb.reward && ra.terminated == rb.terminated &&
                          ra.truncated == rb.truncated &&
                          a.robot().position.x == b.robot().position.x &&
                          a.robot().position.y == b.robot().position.y &&
                          a.robot().heading == b.robot().heading;
        if (!same) {
            ++mismatches;
        }
        if (a.done()) {
            ++episodes;
            a.reset();
        }
        if (b.done()) {
            b.reset();
        }
    }
    CHECK(mismatches == 0);
    // The random policy should end many episodes, exercising resets too.
    CHECK(episodes > 10);
}