#include "robotsim/world.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
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

    const auto beams = static_cast<std::size_t>(cfg_.lidar_beams);
    beam_cos_.resize(beams);
    beam_sin_.resize(beams);
    for (std::size_t i = 0; i < beams; ++i) {
        const double angle = kTwoPi * static_cast<double>(i) / static_cast<double>(beams);
        beam_cos_[i] = std::cos(angle);
        beam_sin_[i] = std::sin(angle);
    }
    map_diagonal_ = std::hypot(w, h);

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

void World::scan_lidar(double* out) const {
    const Vec2 origin = robot_.position;
    const double c = std::cos(robot_.heading);
    const double s = std::sin(robot_.heading);
    const std::size_t beams = beam_cos_.size();

    for (std::size_t i = 0; i < beams; ++i) {
        // Rotate the robot-frame beam direction into the world frame.
        const Vec2 dir{beam_cos_[i] * c - beam_sin_[i] * s, beam_sin_[i] * c + beam_cos_[i] * s};
        double range = cfg_.lidar_range;
        for (const AABB& box : obstacles_) {
            const std::optional<double> hit = ray_aabb(origin, dir, box);
            if (hit && *hit < range) {
                range = *hit;
            }
        }
        out[i] = range;
    }
}

void World::write_observation(float* out) const {
    const std::size_t beams = beam_cos_.size();

    // Stack buffer for the common case; fall back to the heap for large scans.
    constexpr std::size_t kStackBeams = 256;
    double stack_ranges[kStackBeams];
    std::vector<double> heap_ranges;
    double* ranges = stack_ranges;
    if (beams > kStackBeams) {
        heap_ranges.resize(beams);
        ranges = heap_ranges.data();
    }
    scan_lidar(ranges);

    const double inv_range = 1.0 / cfg_.lidar_range;
    for (std::size_t i = 0; i < beams; ++i) {
        out[i] = static_cast<float>(ranges[i] * inv_range);
    }

    // Goal expressed in the robot frame.
    const Vec2 delta = goal_ - robot_.position;
    const double c = std::cos(robot_.heading);
    const double s = std::sin(robot_.heading);
    const double forward = delta.x * c + delta.y * s;
    const double left = -delta.x * s + delta.y * c;
    const double distance = length(delta);

    double sin_bearing = 0.0;
    double cos_bearing = 1.0;  // a goal at the robot position counts as straight ahead
    if (distance > 0.0) {
        sin_bearing = left / distance;
        cos_bearing = forward / distance;
    }

    out[beams] = static_cast<float>(std::min(distance / map_diagonal_, 1.0));
    out[beams + 1] = static_cast<float>(sin_bearing);
    out[beams + 2] = static_cast<float>(cos_bearing);
    out[beams + 3] = static_cast<float>(robot_.v / cfg_.v_max);
}

std::vector<double> World::lidar() const {
    std::vector<double> ranges(beam_cos_.size());
    scan_lidar(ranges.data());
    return ranges;
}

std::vector<float> World::observation() const {
    std::vector<float> obs(static_cast<std::size_t>(observation_size()));
    write_observation(obs.data());
    return obs;
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