#pragma once

#include <cstdint>
#include <vector>

#include "robotsim/geometry.hpp"

namespace robotsim {

// All parameters of the environment. Units are SI (m, s, rad).
struct Config {
    // World
    double map_width = 10.0;
    double map_height = 10.0;
    // Interior obstacles. Boundary walls are added automatically by World.
    std::vector<AABB> obstacles = default_obstacles();

    // Time
    double dt = 0.1;
    int max_steps = 500;

    // Robot
    double robot_radius = 0.2;
    double v_max = 1.0;
    double omega_max = 1.5;

    // Sensor
    int lidar_beams = 36;
    double lidar_range = 5.0;

    // Task
    double goal_radius = 0.3;
    double min_start_goal_distance = 2.0;
    // Extra free space required around the robot and the goal when spawning.
    double spawn_clearance = 0.1;
    int max_spawn_attempts = 10000;

    // Reward (v1)
    double reward_progress = 1.0;
    double reward_step = 0.01;
    double reward_goal = 10.0;
    double reward_collision = 10.0;

    // Fixed warehouse layout used in Phases 1-4: six shelves in three rows
    // and four pillars, leaving aisles of at least 1 m.
    static std::vector<AABB> default_obstacles();
};

// Throws std::invalid_argument if a parameter is out of range.
void validate(const Config& cfg);

}  // namespace robotsim