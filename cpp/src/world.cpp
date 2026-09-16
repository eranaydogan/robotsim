#include "robotsim/world.hpp"

#include <stdexcept>
#include <utility>

namespace robotsim {

namespace {

// Walls are thick boxes just outside the map so that circle and ray tests
// treat them like any other obstacle.
constexpr double kWallThickness = 1.0;

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
        return;
    }
    throw std::runtime_error("robotsim::World: could not find a valid start/goal pair");
}

}  // namespace robotsim