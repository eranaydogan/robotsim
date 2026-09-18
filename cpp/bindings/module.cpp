#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <tuple>
#include <utility>
#include <vector>

#include "robotsim/build_info.hpp"
#include "robotsim/config.hpp"
#include "robotsim/geometry.hpp"
#include "robotsim/platform.hpp"
#include "robotsim/world.hpp"

namespace py = pybind11;

namespace {

using Obstacle = std::tuple<double, double, double, double>;

std::vector<Obstacle> get_obstacles(const robotsim::Config& cfg) {
    std::vector<Obstacle> boxes;
    boxes.reserve(cfg.obstacles.size());
    for (const robotsim::AABB& box : cfg.obstacles) {
        boxes.emplace_back(box.lo.x, box.lo.y, box.hi.x, box.hi.y);
    }
    return boxes;
}

void set_obstacles(robotsim::Config& cfg, const std::vector<Obstacle>& boxes) {
    cfg.obstacles.clear();
    cfg.obstacles.reserve(boxes.size());
    for (const Obstacle& box : boxes) {
        cfg.obstacles.push_back(
            robotsim::AABB{robotsim::Vec2{std::get<0>(box), std::get<1>(box)},
                           robotsim::Vec2{std::get<2>(box), std::get<3>(box)}});
    }
}

py::array_t<float> observation_array(const robotsim::World& world) {
    py::array_t<float> obs(static_cast<py::ssize_t>(world.observation_size()));
    world.write_observation(obs.mutable_data());
    return obs;
}

py::array_t<double> lidar_array(const robotsim::World& world) {
    py::array_t<double> ranges(static_cast<py::ssize_t>(world.config().lidar_beams));
    world.scan_lidar(ranges.mutable_data());
    return ranges;
}

}  // namespace

PYBIND11_MODULE(_robotsim, m) {
    m.doc() = "robotsim C++ core";

    m.def("build_info", &robotsim::build_info,
          "Compiler and build configuration of the C++ core.");
    m.def("pin_to_performance_core", &robotsim::pin_to_performance_core,
          "Pins the calling thread to one performance core for stable benchmark "
          "timings. Returns a description, or an empty string if unavailable.");

    py::class_<robotsim::Config>(m, "Config",
                                 "Parameters of the environment; SI units (m, s, rad).")
        .def(py::init<>())
        .def_readwrite("map_width", &robotsim::Config::map_width)
        .def_readwrite("map_height", &robotsim::Config::map_height)
        .def_property("obstacles", &get_obstacles, &set_obstacles,
                      "Interior obstacles as (x_lo, y_lo, x_hi, y_hi) tuples.")
        .def_readwrite("dt", &robotsim::Config::dt)
        .def_readwrite("max_steps", &robotsim::Config::max_steps)
        .def_readwrite("robot_radius", &robotsim::Config::robot_radius)
        .def_readwrite("v_max", &robotsim::Config::v_max)
        .def_readwrite("omega_max", &robotsim::Config::omega_max)
        .def_readwrite("lidar_beams", &robotsim::Config::lidar_beams)
        .def_readwrite("lidar_range", &robotsim::Config::lidar_range)
        .def_readwrite("goal_radius", &robotsim::Config::goal_radius)
        .def_readwrite("min_start_goal_distance", &robotsim::Config::min_start_goal_distance)
        .def_readwrite("spawn_clearance", &robotsim::Config::spawn_clearance)
        .def_readwrite("max_spawn_attempts", &robotsim::Config::max_spawn_attempts)
        .def_readwrite("reward_progress", &robotsim::Config::reward_progress)
        .def_readwrite("reward_step", &robotsim::Config::reward_step)
        .def_readwrite("reward_goal", &robotsim::Config::reward_goal)
        .def_readwrite("reward_collision", &robotsim::Config::reward_collision)
        .def("validate", [](const robotsim::Config& cfg) { robotsim::validate(cfg); },
             "Raises ValueError if a parameter is out of range.");

    py::class_<robotsim::StepResult>(m, "StepResult", "Outcome of a single simulation step.")
        .def_readonly("reward", &robotsim::StepResult::reward)
        .def_readonly("terminated", &robotsim::StepResult::terminated)
        .def_readonly("truncated", &robotsim::StepResult::truncated)
        .def_readonly("is_success", &robotsim::StepResult::is_success)
        .def_readonly("collision", &robotsim::StepResult::collision);

    py::class_<robotsim::World>(m, "World", "A single navigation environment.")
        .def(py::init<robotsim::Config>(), py::arg("config") = robotsim::Config{})
        .def("reset", py::overload_cast<std::uint64_t>(&robotsim::World::reset), py::arg("seed"),
             "Starts a new episode and restarts the random stream.")
        .def("reset", py::overload_cast<>(&robotsim::World::reset),
             "Starts a new episode, continuing the random stream.")
        .def("step", &robotsim::World::step, py::arg("v"), py::arg("omega"),
             "Advances the simulation by one time step.")
        .def(
            "set_episode",
            [](robotsim::World& world, double x, double y, double heading, double goal_x,
               double goal_y) {
                robotsim::RobotState state;
                state.position = robotsim::Vec2{x, y};
                state.heading = heading;
                world.set_episode(state, robotsim::Vec2{goal_x, goal_y});
            },
            py::arg("x"), py::arg("y"), py::arg("heading"), py::arg("goal_x"), py::arg("goal_y"),
            "Places the robot and goal directly and starts a new episode.")
        .def("observation", &observation_array, "Normalised observation as a float32 array.")
        .def("lidar", &lidar_array, "LiDAR ranges in metres as a float64 array.")
        .def_property_readonly("config", &robotsim::World::config)
        .def_property_readonly("observation_size", &robotsim::World::observation_size)
        .def_property_readonly("step_count", &robotsim::World::step_count)
        .def_property_readonly("done", &robotsim::World::done)
        .def_property_readonly("position",
                               [](const robotsim::World& w) {
                                   return std::make_pair(w.robot().position.x,
                                                         w.robot().position.y);
                               })
        .def_property_readonly("heading",
                               [](const robotsim::World& w) { return w.robot().heading; })
        .def_property_readonly("v", [](const robotsim::World& w) { return w.robot().v; })
        .def_property_readonly("omega", [](const robotsim::World& w) { return w.robot().omega; })
        .def_property_readonly(
            "goal",
            [](const robotsim::World& w) { return std::make_pair(w.goal().x, w.goal().y); })
        .def_property_readonly("obstacles", [](const robotsim::World& w) {
            std::vector<Obstacle> boxes;
            boxes.reserve(w.obstacles().size());
            for (const robotsim::AABB& box : w.obstacles()) {
                boxes.emplace_back(box.lo.x, box.lo.y, box.hi.x, box.hi.y);
            }
            return boxes;
        });
}