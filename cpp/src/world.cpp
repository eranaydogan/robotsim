#include "robotsim/world.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robotsim {

namespace {

// Walls are thick boxes just outside the map so that circle and ray tests
// treat them like any other obstacle.
constexpr double kWallThickness = 1.0;

// Below this angular velocity the motion is integrated as a straight line to
// avoid dividing by a value close to zero. The position error is of order
// v * omega * dt^2, far below the test tolerances.
constexpr double kStraightLineOmega = 1e-9;

}  // namespace

World::World(Config cfg) : cfg_(std::move(cfg)) {
    validate(cfg_);

    const double w = cfg_.map_width;
    const double h = cfg_.map_height;
    const double t = kWallThickness;

    obstacles_ = cfg_.obstacles;
    obstacles_.push_back(AABB{Vec2{-t, -t}, Vec2{0.0, h + t}});    // left
    obstacles_.push_back(AABB{Vec2{w, -t}, Vec2{w + t, h + t}});   // right
    obstacles_.push_back(AABB{Vec2{-t, -t}, Vec2{w + t, 0.0}});    // bottom
    obstacles_.push_back(AABB{Vec2{-t, h}, Vec2{w + t, h + t}});   // top

    reset(0);
}

bool World::in_collision(Vec2 p) const {
    for (const AABB& box : obstacles_) {
        if (circle_intersects_aabb(p, cfg_.robot_radius, box)) {
            return true;
        }
    }
    return false;
}

StepResult World::step(double v, double omega) {
    if (!std::isfinite(v) || !std::isfinite(omega)) {
        throw std::invalid_argument("robotsim::World::step: commands must be finite");
    }
    if (done_) {
        throw std::logic_error("robotsim::World::step: episode has ended, call reset()");
    }

    v = std::clamp(v, 0.0, cfg_.v_max);
    omega = std::clamp(omega, -cfg_.omega_max, cfg_.omega_max);

    const double dt = cfg_.dt;
    const double distance_before = length(goal_ - robot_.position);
    const double theta = robot_.heading;

    // Exact integration of unicycle kinematics for constant (v, omega) over dt.
    if (std::abs(omega) < kStraightLineOmega) {
        robot_.position.x += v * std::cos(theta) * dt;
        robot_.position.y += v * std::sin(theta) * dt;
    } else {
        const double theta_next = theta + omega * dt;
        const double r = v / omega;
        robot_.position.x += r * (std::sin(theta_next) - std::sin(theta));
        robot_.position.y -= r * (std::cos(theta_next) - std::cos(theta));
    }
    robot_.heading = wrap_angle(theta + omega * dt);
    robot_.v = v;
    robot_.omega = omega;
    ++step_count_;

    StepResult result;
    const double distance_after = length(goal_ - robot_.position);
    result.collision = in_collision(robot_.position);
    // A collision takes priority: touching an obstacle is never a success.
    result.is_success = !result.collision && distance_after <= cfg_.goal_radius;
    result.terminated = result.collision || result.is_success;
    result.truncated = !result.terminated && step_count_ >= cfg_.max_steps;

    result.reward = cfg_.reward_progress * (distance_before - distance_after) - cfg_.reward_step;
    if (result.is_success) {
        result.reward += cfg_.reward_goal;
    }
    if (result.collision) {
        result.reward -= cfg_.reward_collision;
    }

    done_ = result.terminated || result.truncated;
    return result;
}

void World::set_episode(const RobotState& robot, Vec2 goal) {
    robot_ = robot;
    robot_.heading = wrap_angle(robot.heading);
    goal_ = goal;
    step_count_ = 0;
    done_ = false;
}

bool World::is_free(Vec2 p, double radius) const {
    if (p.x < radius || p.y < radius || p.x > cfg_.map_width - radius ||
        p.y > cfg_.map_height - radius) {
        return false;
    }
    for (const AABB& box : obstacles_) {
        if (circle_intersects_aabb(p, radius, box)) {
            return false;
        }
    }
    return true;
}

Vec2 World::sample_free_point(double radius) {
    for (int attempt = 0; attempt < cfg_.max_spawn_attempts; ++attempt) {
        const Vec2 p{rng_.uniform(radius, cfg_.map_width - radius),
                     rng_.uniform(radius, cfg_.map_height - radius)};
        if (is_free(p, radius)) {
            return p;
        }
    }
    throw std::runtime_error("robotsim::World: could not find a free spawn point");
}

void World::reset(std::uint64_t seed) {
    rng_.reseed(seed);
    reset();
}

void World::reset() {
    const double radius = cfg_.robot_radius + cfg_.spawn_clearance;
    const double min_dist_sq = cfg_.min_start_goal_distance * cfg_.min_start_goal_distance;

    for (int attempt = 0; attempt < cfg_.max_spawn_attempts; ++attempt) {
        const Vec2 start = sample_free_point(radius);
        const Vec2 goal = sample_free_point(radius);
        if (length_squared(goal - start) < min_dist_sq) {
            continue;
        }
        robot_ = RobotState{};
        robot_.position = start;
        robot_.heading = wrap_angle(rng_.uniform(-kPi, kPi));
        goal_ = goal;
        step_count_ = 0;
        done_ = false;
        return;
    }
    throw std::runtime_error("robotsim::World: could not find a valid start/goal pair");
}

}  // namespace robotsim