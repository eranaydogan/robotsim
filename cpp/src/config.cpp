#include "robotsim/config.hpp"

#include <stdexcept>
#include <string>

namespace robotsim {

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::invalid_argument("robotsim::Config: " + message);
    }
}

}  // namespace

std::vector<AABB> Config::default_obstacles() {
    return {
        // Shelves (2.5 m x 0.5 m)
        AABB{Vec2{1.5, 2.0}, Vec2{4.0, 2.5}},
        AABB{Vec2{6.0, 2.0}, Vec2{8.5, 2.5}},
        AABB{Vec2{1.5, 4.5}, Vec2{4.0, 5.0}},
        AABB{Vec2{6.0, 4.5}, Vec2{8.5, 5.0}},
        AABB{Vec2{1.5, 7.0}, Vec2{4.0, 7.5}},
        AABB{Vec2{6.0, 7.0}, Vec2{8.5, 7.5}},
        // Pillars (0.4 m x 0.4 m)
        AABB{Vec2{4.8, 3.3}, Vec2{5.2, 3.7}},
        AABB{Vec2{4.8, 5.8}, Vec2{5.2, 6.2}},
        AABB{Vec2{2.55, 8.4}, Vec2{2.95, 8.8}},
        AABB{Vec2{7.05, 0.8}, Vec2{7.45, 1.2}},
    };
}

void validate(const Config& cfg) {
    require(cfg.map_width > 0.0 && cfg.map_height > 0.0, "map size must be positive");
    require(cfg.dt > 0.0, "dt must be positive");
    require(cfg.max_steps > 0, "max_steps must be positive");
    require(cfg.robot_radius > 0.0, "robot_radius must be positive");
    require(cfg.v_max > 0.0, "v_max must be positive");
    require(cfg.omega_max > 0.0, "omega_max must be positive");
    require(cfg.lidar_beams > 0, "lidar_beams must be positive");
    require(cfg.lidar_range > 0.0, "lidar_range must be positive");
    require(cfg.goal_radius > 0.0, "goal_radius must be positive");
    require(cfg.min_start_goal_distance >= 0.0, "min_start_goal_distance must be non-negative");
    require(cfg.spawn_clearance >= 0.0, "spawn_clearance must be non-negative");
    require(cfg.max_spawn_attempts > 0, "max_spawn_attempts must be positive");
    const double margin = 2.0 * (cfg.robot_radius + cfg.spawn_clearance);
    require(cfg.map_width > margin && cfg.map_height > margin,
            "map is too small for the robot and spawn clearance");
    for (const AABB& box : cfg.obstacles) {
        require(box.lo.x < box.hi.x && box.lo.y < box.hi.y, "obstacle must have lo < hi");
    }
}

}  // namespace robotsim