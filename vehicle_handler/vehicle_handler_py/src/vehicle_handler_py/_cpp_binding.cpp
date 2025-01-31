// Copyright 2024 Maximilian Leitenstern
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "vehicle_handler_cpp/vehicle_handler.hpp"

namespace py = pybind11;
PYBIND11_MODULE(_cpp_binding, m)
{
  py::class_<tam::common::VehicleHandler>(m, "VehicleHandler")
    .def("from_pkg_config", &tam::common::VehicleHandler::from_pkg_config, "")
    .def("from_path", &tam::common::VehicleHandler::from_path, "")
    .def("get_vehicle_name", &tam::common::VehicleHandler::get_vehicle_name, "")
    .def("vehicle", &tam::common::VehicleHandler::vehicle, "")
    .def("tires", &tam::common::VehicleHandler::tires, "")
    .def("list_parameters", &tam::common::VehicleHandler::list_parameters, "")
    .def("init_param_backend", &tam::common::VehicleHandler::init_param_backend, "")
    .def("load_params", &tam::common::VehicleHandler::load_params, "")
    .def("overwrite_param", &tam::common::VehicleHandler::overwrite_param, "")
    .def("calc_aerodynamics", &tam::common::VehicleHandler::calc_aerodynamics, "")
    .def("calc_dynamic_tire_radius", &tam::common::VehicleHandler::calc_dynamic_tire_radius, "")
    .def(
      "calc_long_slip_odometry",
      py::overload_cast<
        const tam::types::control::Odometry &, const double &,
        const tam::types::common::DataPerWheel<double> &>(
        &tam::common::VehicleHandler::calc_long_slip, py::const_),
      "Calculate longitudinal slip with odometry, steering angle, and wheel speeds.")
    .def("calc_long_slip_vectorized", &tam::common::VehicleHandler::calc_long_slip_vectorized, "")
    .def("calc_slip_angles", &tam::common::VehicleHandler::calc_slip_angles, "")
    .def(
      "calc_slip_angles_vectorized", &tam::common::VehicleHandler::calc_slip_angles_vectorized, "");
}
