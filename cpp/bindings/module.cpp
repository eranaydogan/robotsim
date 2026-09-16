#include <pybind11/pybind11.h>

#include "robotsim/example.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_robotsim, m) {
    m.doc() = "robotsim C++ core";

    m.def("add", &robotsim::add, py::arg("a"), py::arg("b"),
          "Add two integers (Phase 0 smoke test).");
}