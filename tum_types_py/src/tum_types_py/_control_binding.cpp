// Copyright 2023 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "tum_types_cpp/control.hpp"

namespace py = pybind11;
namespace typ = tam::types;
//
PYBIND11_MODULE(_control_binding, m)
{
  // #region Tracking Controller
  py::class_<tam::types::control::TrajectoryPoint>(m, "TrajectoryPoint")
    .def(py::init())
    .def_readwrite("position_m", &tam::types::control::TrajectoryPoint::position_m)
    .def_readwrite("orientation_rad", &tam::types::control::TrajectoryPoint::orientation_rad)
    .def_readwrite("velocity_mps", &tam::types::control::TrajectoryPoint::velocity_mps)
    .def_readwrite(
      "angular_velocity_radps", &tam::types::control::TrajectoryPoint::angular_velocity_radps)
    .def_readwrite("acceleration_mps2", &tam::types::control::TrajectoryPoint::acceleration_mps2)
    .def_readwrite(
      "angular_acceleration_radps2",
      &tam::types::control::TrajectoryPoint::angular_acceleration_radps2);
  py::class_<tam::types::control::Trajectory>(m, "Trajectory")
    .def(py::init())
    .def_readwrite("header", &tam::types::control::Trajectory::header)
    .def_readwrite("points", &tam::types::control::Trajectory::points);
  py::class_<tam::types::control::Odometry>(m, "Odometry")
    .def(py::init())
    .def_readwrite("position_m", &tam::types::control::Odometry::position_m)
    .def_readwrite("orientation_rad", &tam::types::control::Odometry::orientation_rad)
    .def_readwrite("pose_covariance", &tam::types::control::Odometry::pose_covariance)
    .def_readwrite("velocity_mps", &tam::types::control::Odometry::velocity_mps)
    .def_readwrite("angular_velocity_radps", &tam::types::control::Odometry::angular_velocity_radps)
    .def_readwrite("velocity_covariance", &tam::types::control::Odometry::velocity_covariance);
  py::class_<tam::types::control::AccelerationwithCovariances>(m, "AccelerationwithCovariances")
    .def(py::init())
    .def_readwrite(
      "acceleration_mps2", &tam::types::control::AccelerationwithCovariances::acceleration_mps2)
    .def_readwrite(
      "angular_acceleration_radps2",
      &tam::types::control::AccelerationwithCovariances::angular_acceleration_radps2)
    .def_readwrite(
      "acceleration_covariance",
      &tam::types::control::AccelerationwithCovariances::acceleration_covariance);
  py::class_<tam::types::control::AutowareSteeringReport>(m, "AutowareSteeringReport")
    .def(py::init())
    .def_readwrite("time_stamp_ns", &tam::types::control::AutowareSteeringReport::time_stamp_ns)
    .def_readwrite(
      "steering_angle_tire_rad",
      &tam::types::control::AutowareSteeringReport::steering_angle_tire_rad);
  py::enum_<tam::types::control::AutowareOperationMode>(m, "AutowareOperationMode")
    .value("UNKNOWN", tam::types::control::AutowareOperationMode::UNKNOWN)
    .value("STOP", tam::types::control::AutowareOperationMode::STOP)
    .value("AUTONOMOUS", tam::types::control::AutowareOperationMode::AUTONOMOUS)
    .value("LOCAL", tam::types::control::AutowareOperationMode::LOCAL)
    .value("REMOTE", tam::types::control::AutowareOperationMode::REMOTE);
  py::class_<tam::types::control::AutowareOperationModeState>(m, "AutowareOperationModeState")
    .def(py::init())
    .def_readwrite("time_stamp_ns", &tam::types::control::AutowareOperationModeState::time_stamp_ns)
    .def_readwrite("mode", &tam::types::control::AutowareOperationModeState::mode)
    .def_readwrite(
      "is_autoware_control_enabled",
      &tam::types::control::AutowareOperationModeState::is_autoware_control_enabled)
    .def_readwrite(
      "is_in_transition", &tam::types::control::AutowareOperationModeState::is_in_transition)
    .def_readwrite(
      "is_stop_mode_available",
      &tam::types::control::AutowareOperationModeState::is_stop_mode_available)
    .def_readwrite(
      "is_autonomous_mode_available",
      &tam::types::control::AutowareOperationModeState::is_autonomous_mode_available)
    .def_readwrite(
      "is_local_mode_available",
      &tam::types::control::AutowareOperationModeState::is_local_mode_available)
    .def_readwrite(
      "is_remote_mode_available",
      &tam::types::control::AutowareOperationModeState::is_remote_mode_available);
  py::class_<tam::types::control::LongitudinalControlCommand>(m, "LongitudinalControlCommand")
    .def(py::init())
    .def_readwrite("time_stamp_ns", &tam::types::control::LongitudinalControlCommand::time_stamp_ns)
    .def_readwrite(
      "control_time_ns", &tam::types::control::LongitudinalControlCommand::control_time_ns)
    .def_readwrite("velocity_mps", &tam::types::control::LongitudinalControlCommand::velocity_mps)
    .def_readwrite(
      "acceleration_mps2", &tam::types::control::LongitudinalControlCommand::acceleration_mps2)
    .def_readwrite("jerk_mps3", &tam::types::control::LongitudinalControlCommand::jerk_mps3)
    .def_readwrite(
      "is_defined_acceleration",
      &tam::types::control::LongitudinalControlCommand::is_defined_acceleration)
    .def_readwrite(
      "is_defined_jerk", &tam::types::control::LongitudinalControlCommand::is_defined_jerk);
  py::class_<tam::types::control::LateralControlCommand>(m, "LateralControlCommand")
    .def(py::init())
    .def_readwrite("time_stamp_ns", &tam::types::control::LateralControlCommand::time_stamp_ns)
    .def_readwrite("control_time_ns", &tam::types::control::LateralControlCommand::control_time_ns)
    .def_readwrite(
      "steering_angle_tire_rad",
      &tam::types::control::LateralControlCommand::steering_angle_tire_rad)
    .def_readwrite(
      "steering_rotation_rate_tire_radps",
      &tam::types::control::LateralControlCommand::steering_rotation_rate_tire_radps)
    .def_readwrite(
      "is_defined_steering_rotation_rate",
      &tam::types::control::LateralControlCommand::is_defined_steering_rotation_rate);
  py::class_<tam::types::control::ControlConstraintPoint>(m, "ControlConstraintPoint")
    .def(py::init())
    .def_readwrite("a_x_min_mps2", &tam::types::control::ControlConstraintPoint::a_x_min_mps2)
    .def_readwrite("a_x_max_mps2", &tam::types::control::ControlConstraintPoint::a_x_max_mps2)
    .def_readwrite("a_y_min_mps2", &tam::types::control::ControlConstraintPoint::a_y_min_mps2)
    .def_readwrite("a_y_max_mps2", &tam::types::control::ControlConstraintPoint::a_y_max_mps2)
    .def_readwrite(
      "a_x_max_engine_mps2", &tam::types::control::ControlConstraintPoint::a_x_max_engine_mps2)
    .def_readwrite(
      "shape_factor",
      &tam::types::control::ControlConstraintPoint::shape_factor)
    .def_readwrite(
      "lateral_error_min_m", &tam::types::control::ControlConstraintPoint::lateral_error_min_m)
    .def_readwrite(
      "lateral_error_max_m", &tam::types::control::ControlConstraintPoint::lateral_error_max_m);
  py::class_<tam::types::control::ControlConstraints>(m, "ControlConstraints")
    .def(py::init())
    .def_readwrite("header", &tam::types::control::ControlConstraints::header)
    .def_readwrite("points", &tam::types::control::ControlConstraints::points);
  py::class_<tam::types::control::AdditionalInfoPoint>(m, "AdditionalInfoPoint")
    .def(py::init())
    .def_readwrite("s_global_m", &tam::types::control::AdditionalInfoPoint::s_global_m)
    .def_readwrite("s_local_m", &tam::types::control::AdditionalInfoPoint::s_local_m)
    .def_readwrite("kappa_1pm", &tam::types::control::AdditionalInfoPoint::kappa_1pm)
    .def_readwrite("lap_cnt", &tam::types::control::AdditionalInfoPoint::lap_cnt);
  py::class_<tam::types::control::AdditionalTrajectoryInfos>(m, "AdditionalTrajectoryInfos")
    .def(py::init())
    .def_readwrite("header", &tam::types::control::AdditionalTrajectoryInfos::header)
    .def_readwrite("points", &tam::types::control::AdditionalTrajectoryInfos::points);
  // #endregion
  // #region Lat Controller
  py::class_<tam::types::control::ICECommand>(m, "ICECommand")
    .def(py::init())
    .def_readwrite("throttle", &tam::types::control::ICECommand::throttle)
    .def_readwrite("brake_pressure_Pa", &tam::types::control::ICECommand::brake_pressure_Pa)
    .def_readwrite("gear", &tam::types::control::ICECommand::gear);
  py::class_<tam::types::control::DriveTrainFeedback>(m, "DriveTrainFeedback")
    .def(py::init())
    .def_readwrite(
      "omega_engine_radps", &tam::types::control::DriveTrainFeedback::omega_engine_radps)
    .def_readwrite("gear_engaged", &tam::types::control::DriveTrainFeedback::gear_engaged);
  // #endregion
}
