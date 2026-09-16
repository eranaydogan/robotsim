#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "robotsim/build_info.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_robotsim, m) {
    m.doc() = "robotsim C++ core";

    m.def("build_info", &robotsim::build_info,
          "Compiler and build configuration of the C++ core.");
}