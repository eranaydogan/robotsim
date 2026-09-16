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

struct StepResult {
    double reward = 0.0;
    bool terminated = false;  // goal reached or collision
    bool truncated = false;   // max_steps reached without termination
    bool is_success = false;
    bool collision = false;
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

    // Advances the simulation by one time step with the commanded linear
    // velocity v (m/s) and angular velocity omega (rad/s). Commands are clamped
    // to [0, v_max] and [-omega_max, omega_max].
    // Throws std::invalid_argument for non-finite commands and std::logic_error
    // if the episode has already ended (call reset first).
    StepResult step(double v, double omega);

    // Sets the robot state and goal directly and starts a new episode from
    // them. Used by tests and episode replay; no validity checks are made.
    void set_episode(const RobotState& robot, Vec2 goal);

    // True if a circle of the given radius at p lies inside the map and does
    // not touch any obstacle or wall.
    bool is_free(Vec2 p, double radius) const;

    const Config& config() const { return cfg_; }
    const RobotState& robot() const { return robot_; }
    Vec2 goal() const { return goal_; }
    int step_count() const { return step_count_; }
    bool done() const { return done_; }

    // Interior obstacles followed by the four boundary walls.
    const std::vector<AABB>& obstacles() const { return obstacles_; }

private:
    Vec2 sample_free_point(double radius);
    bool in_collision(Vec2 p) const;

    Config cfg_;
    std::vector<AABB> obstacles_;
    Rng rng_;
    RobotState robot_{};
    Vec2 goal_{};
    int step_count_ = 0;
    bool done_ = false;
};

}  // namespace robotsim