// Copyright 2024 Maximilian Leitenstern
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "tum_types_cpp/vehicle.hpp"

namespace py = pybind11;
namespace typ = tam::types;
//
PYBIND11_MODULE(_vehicle_binding, m)
{
  py::class_<tam::types::vehicle_params::Actuator>(m, "Actuator")
    .def(py::init())
    .def_readwrite("brake_delay", &tam::types::vehicle_params::Actuator::brake_delay)
    .def_readwrite("throttle_delay", &tam::types::vehicle_params::Actuator::throttle_delay)
    .def_readwrite("steering_delay", &tam::types::vehicle_params::Actuator::steering_delay)
    .def_readwrite("gear_delay", &tam::types::vehicle_params::Actuator::gear_delay);

  py::class_<tam::types::vehicle_params::Aero>(m, "Aero")
    .def(py::init())
    .def_readwrite("air_density", &tam::types::vehicle_params::Aero::air_density)
    .def_readwrite("drag_coeff", &tam::types::vehicle_params::Aero::drag_coeff)
    .def_readwrite("lift_coeff", &tam::types::vehicle_params::Aero::lift_coeff)
    .def_readwrite("cross_track_area", &tam::types::vehicle_params::Aero::cross_track_area)
    .def_readwrite("diff_cog_x", &tam::types::vehicle_params::Aero::diff_cog_x)
    .def_readwrite("diff_cog_y", &tam::types::vehicle_params::Aero::diff_cog_y)
    .def_readwrite("diff_cog_z", &tam::types::vehicle_params::Aero::diff_cog_z);

  py::class_<tam::types::vehicle_params::AeroModelOutput>(m, "AeroModelOutput")
    .def(py::init())
    .def_readwrite("force_cog", &tam::types::vehicle_params::AeroModelOutput::force_cog)
    .def_readwrite("torque", &tam::types::vehicle_params::AeroModelOutput::torque);

  py::class_<tam::types::vehicle_params::Brake>(m, "Brake")
    .def(py::init())
    .def_readwrite("brake_bias_front", &tam::types::vehicle_params::Brake::brake_bias_front)
    .def_readwrite("disc_area", &tam::types::vehicle_params::Brake::disc_area)
    .def_readwrite("disc_radius", &tam::types::vehicle_params::Brake::disc_radius)
    .def_readwrite("pad_area", &tam::types::vehicle_params::Brake::pad_area)
    .def_readwrite("max_pressure", &tam::types::vehicle_params::Brake::max_pressure)
    .def_readwrite("friction_coeff", &tam::types::vehicle_params::Brake::friction_coeff);

  py::class_<tam::types::vehicle_params::Dimension>(m, "Dimension")
    .def(py::init())
    .def_readwrite("track_width_front", &tam::types::vehicle_params::Dimension::track_width_front)
    .def_readwrite("track_width_rear", &tam::types::vehicle_params::Dimension::track_width_rear)
    .def_readwrite("wheelbase", &tam::types::vehicle_params::Dimension::wheelbase)
    .def_readwrite(
      "distance_to_front_axle", &tam::types::vehicle_params::Dimension::distance_to_front_axle)
    .def_readwrite("cog_height", &tam::types::vehicle_params::Dimension::cog_height)
    .def_readwrite("length", &tam::types::vehicle_params::Dimension::length)
    .def_readwrite("width", &tam::types::vehicle_params::Dimension::width)
    .def_readwrite("height", &tam::types::vehicle_params::Dimension::height);

  py::class_<tam::types::vehicle_params::Drivetrain>(m, "Drivetrain")
    .def(py::init())
    .def_readwrite(
      "clutch_engagement_start", &tam::types::vehicle_params::Drivetrain::clutch_engagement_start)
    .def_readwrite(
      "clutch_max_torque_slope", &tam::types::vehicle_params::Drivetrain::clutch_max_torque_slope)
    .def_readwrite(
      "drivetrain_efficiency", &tam::types::vehicle_params::Drivetrain::drivetrain_efficiency)
    .def_readwrite(
      "transmission_ratio", &tam::types::vehicle_params::Drivetrain::transmission_ratio)
    .def_readwrite("gear_ratios", &tam::types::vehicle_params::Drivetrain::gear_ratios);

  py::class_<tam::types::vehicle_params::EngineMap>(m, "EngineMap")
    .def(py::init())
    .def_readwrite("rpms", &tam::types::vehicle_params::EngineMap::rpms)
    .def_readwrite("throttles", &tam::types::vehicle_params::EngineMap::throttles)
    .def_readwrite("torques", &tam::types::vehicle_params::EngineMap::torques);

  py::class_<tam::types::vehicle_params::Engine>(m, "Engine")
    .def(py::init())
    .def_readwrite("rev_idle", &tam::types::vehicle_params::Engine::rev_idle)
    .def_readwrite("rev_max", &tam::types::vehicle_params::Engine::rev_max)
    .def_readwrite("engine_map", &tam::types::vehicle_params::Engine::engine_map);

  py::class_<tam::types::vehicle_params::Inertia>(m, "Inertia")
    .def(py::init())
    .def_readwrite("roll", &tam::types::vehicle_params::Inertia::roll)
    .def_readwrite("pitch", &tam::types::vehicle_params::Inertia::pitch)
    .def_readwrite("yaw", &tam::types::vehicle_params::Inertia::yaw)
    .def_readwrite("engine", &tam::types::vehicle_params::Inertia::engine)
    .def_readwrite("wheel_front", &tam::types::vehicle_params::Inertia::wheel_front)
    .def_readwrite("wheel_rear", &tam::types::vehicle_params::Inertia::wheel_rear);

  py::class_<tam::types::vehicle_params::Steering>(m, "Steering")
    .def(py::init())
    .def_readwrite("min_angle", &tam::types::vehicle_params::Steering::min_angle)
    .def_readwrite("max_angle", &tam::types::vehicle_params::Steering::max_angle)
    .def_readwrite("max_rate", &tam::types::vehicle_params::Steering::max_rate)
    .def_readwrite("ratio", &tam::types::vehicle_params::Steering::ratio);

  py::class_<tam::types::vehicle_params::Mass>(m, "Mass")
    .def(py::init())
    .def_readwrite("total", &tam::types::vehicle_params::Mass::total)
    .def_readwrite("wheel_front", &tam::types::vehicle_params::Mass::wheel_front)
    .def_readwrite("wheel_rear", &tam::types::vehicle_params::Mass::wheel_rear);

  py::class_<tam::types::vehicle_params::Vehicle>(m, "Vehicle")
    .def(py::init())
    .def_readwrite("actuator", &tam::types::vehicle_params::Vehicle::actuator)
    .def_readwrite("aero", &tam::types::vehicle_params::Vehicle::aero)
    .def_readwrite("brake", &tam::types::vehicle_params::Vehicle::brake)
    .def_readwrite("dimension", &tam::types::vehicle_params::Vehicle::dimension)
    .def_readwrite("drivetrain", &tam::types::vehicle_params::Vehicle::drivetrain)
    .def_readwrite("engine", &tam::types::vehicle_params::Vehicle::engine)
    .def_readwrite("inertia", &tam::types::vehicle_params::Vehicle::inertia)
    .def_readwrite("steering", &tam::types::vehicle_params::Vehicle::steering)
    .def_readwrite("mass", &tam::types::vehicle_params::Vehicle::mass);
}