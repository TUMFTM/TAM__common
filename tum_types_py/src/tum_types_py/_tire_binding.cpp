// Copyright 2024 Maximilian Leitenstern
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "tum_types_cpp/tire.hpp"

namespace py = pybind11;
namespace typ = tam::types;
//
PYBIND11_MODULE(_tire_binding, m)
{
  py::class_<tam::types::tire_params::VelocityScaling>(m, "VelocityScaling")
    .def(py::init())
    .def_readwrite("velocity", &tam::types::tire_params::VelocityScaling::velocity)
    .def_readwrite("factor", &tam::types::tire_params::VelocityScaling::factor);
  py::class_<tam::types::tire_params::Radius>(m, "Radius")
    .def(py::init())
    .def_readwrite("radius_20mps", &tam::types::tire_params::Radius::radius_20mps)
    .def_readwrite("velocity_scaling", &tam::types::tire_params::Radius::velocity_scaling);
  py::class_<tam::types::tire_params::Setup>(m, "Setup")
    .def(py::init())
    .def_readwrite("camber", &tam::types::tire_params::Setup::camber)
    .def_readwrite("caster", &tam::types::tire_params::Setup::caster)
    .def_readwrite("toe", &tam::types::tire_params::Setup::toe);
  py::class_<tam::types::tire_params::Tire>(m, "Tire")
    .def(py::init())
    .def_readwrite("radius", &tam::types::tire_params::Tire::radius)
    .def_readwrite("setup", &tam::types::tire_params::Tire::setup);
}
