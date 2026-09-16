#pragma once

#include <cstdint>
#include <vector>

#include "robotsim/config.hpp"
#include "robotsim/geometry.hpp"
#include "robotsim/rng.hpp"

namespace robotsim {

struct RobotState {
    Vec2 position{};
    double heading = 0.0;  // radians in [-pi, pi), 0 faces +x
    double v = 0.0;        // last applied linear velocity (m/s)
    double omega = 0.0;    // last applied angular velocity (rad/s)
};

// A single navigation environment.
class World {
public:
    // Validates the configuration. Throws std::invalid_argument on bad values.
    explicit World(Config cfg);

    // Starts a new episode. The seeded overload restarts the random stream;
    // the unseeded overload continues it. Throws std::runtime_error if no
    // valid start/goal pair is found within cfg.max_spawn_attempts.
    void reset(std::uint64_t seed);
    void reset();

    // True if a circle of the given radius at p lies inside the map and does
    // not touch any obstacle or wall.
    bool is_free(Vec2 p, double radius) const;

    const Config& config() const { return cfg_; }
    const RobotState& robot() const { return robot_; }
    Vec2 goal() const { return goal_; }
    int step_count() const { return step_count_; }

    // Interior obstacles followed by the four boundary walls.
    const std::vector<AABB>& obstacles() const { return obstacles_; }

private:
    Vec2 sample_free_point(double radius);

    Config cfg_;
    std::vector<AABB> obstacles_;
    Rng rng_;
    RobotState robot_{};
    Vec2 goal_{};
    int step_count_ = 0;
};

}  // namespace robotsim