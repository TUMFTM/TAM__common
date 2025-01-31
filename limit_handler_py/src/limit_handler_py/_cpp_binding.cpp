// Copyright 2023 Simon Hoffmann
#include <eigen3/Eigen/Dense>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"


#include "limit_handler_cpp/helpers.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
// #include <pybind11/stl.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Vector3.h>
// // include "pybind11_test/test_struct.hpp"

namespace py = pybind11;
PYBIND11_MODULE(_cpp_binding, m)
{
  py::class_<tam::limits::Margin>(m, "Margin")
    .def(py::init())
    .def(py::init<float>())
    .def(py::init<float, float, float, float>())
    .def_readwrite("ax_max", &tam::limits::Margin::ax_max)
    .def_readwrite("ax_min", &tam::limits::Margin::ax_min)
    .def_readwrite("ay_min", &tam::limits::Margin::ay_min)
    .def_readwrite("ay_max", &tam::limits::Margin::ay_max);
  py::class_<tam::limits::Scaling>(m, "Scaling")
    .def(py::init())
    .def(py::init<float>())
    .def(py::init<float, float, float, float>())
    .def_readwrite("ax_max", &tam::limits::Scaling::ax_max)
    .def_readwrite("ax_min", &tam::limits::Scaling::ax_min)
    .def_readwrite("ay_min", &tam::limits::Scaling::ay_min)
    .def_readwrite("ay_max", &tam::limits::Scaling::ay_max);
  m.def("scale_constraint_point", &tam::limits::scale_constraint_point)
  .def("pt_within_limits", &tam::limits::pt_within_limits)
  .def("get_max_ax_from_ay", &tam::limits::get_max_ax_from_ay);
}
