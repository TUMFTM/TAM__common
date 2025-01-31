// // Copyright 2023 Simon Sagmeister
#include <eigen3/Eigen/Dense>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

// #include "tum_helpers_cpp/geometry/geometry.hpp"

#include "tum_helpers_cpp/aerodynamics.hpp"
#include "tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp"
#include "tum_helpers_cpp/delay_compensation.hpp"
#include "tum_helpers_cpp/geometry/geometry.hpp"
#include "tum_helpers_cpp/rotations.hpp"
#include "tum_helpers_cpp/vehicle_dynamics.hpp"
// #include <pybind11/stl.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Vector3.h>
// // include "pybind11_test/test_struct.hpp"

namespace py = pybind11;
PYBIND11_MODULE(_cpp_binding, m)
{
  py::class_<tam::helpers::cosy::CurvilinearCosy>(m, "CurvilinearCosy")
    .def(
      "convert_to_sn", py::overload_cast<const double, const double>(
                         &tam::helpers::cosy::CurvilinearCosy::convert_to_sn, py::const_))
    .def(
      "convert_to_sn", py::overload_cast<const double, const double, const double>(
                         &tam::helpers::cosy::CurvilinearCosy::convert_to_sn, py::const_))
    .def(
      "convert_to_sn", py::overload_cast<const tam::types::control::Odometry &>(
                         &tam::helpers::cosy::CurvilinearCosy::convert_to_sn, py::const_))
    .def(
      "convert_to_sn_and_get_idx",
      py::overload_cast<const double, const double>(
        &tam::helpers::cosy::CurvilinearCosy::convert_to_sn_and_get_idx, py::const_))
    .def(
      "convert_to_sn_and_get_idx",
      py::overload_cast<const double, const double, const double>(
        &tam::helpers::cosy::CurvilinearCosy::convert_to_sn_and_get_idx, py::const_))
    .def(
      "convert_to_sn_and_get_idx",
      py::overload_cast<const tam::types::control::Odometry &>(
        &tam::helpers::cosy::CurvilinearCosy::convert_to_sn_and_get_idx, py::const_))

    .def(
      "convert_to_cartesian",
      py::overload_cast<const double, const double>(
        &tam::helpers::cosy::CurvilinearCosy::convert_to_cartesian, py::const_))
    .def(
      "convert_to_cartesian",
      py::overload_cast<const double, const double, const double>(
        &tam::helpers::cosy::CurvilinearCosy::convert_to_cartesian, py::const_));
  py::class_<tam::helpers::cosy::CurvilinearCosyBuilder>(m, "CurvilinearCosyBuilder")
    .def(py::init<const tam::types::control::Trajectory &>())
    .def(py::init<const Eigen::VectorXd &, const Eigen::VectorXd &, const Eigen::VectorXd &>())
    .def("build", &tam::helpers::cosy::CurvilinearCosyBuilder::build, "")
    .def(
      "set_s",
      py::overload_cast<Eigen::VectorXd &>(&tam::helpers::cosy::CurvilinearCosyBuilder::set_s), "")
    .def(
      "set_s",
      py::overload_cast<const tam::types::control::AdditionalTrajectoryInfos &>(
        &tam::helpers::cosy::CurvilinearCosyBuilder::set_s),
      "")
    .def(
      "set_tangent",
      py::overload_cast<const Eigen::VectorXd &, const Eigen::VectorXd &, const Eigen::VectorXd &>(
        &tam::helpers::cosy::CurvilinearCosyBuilder::set_tangent))
    .def(
      "set_tangent", py::overload_cast<const Eigen::Ref<const Eigen::MatrixX3d>>(
                       &tam::helpers::cosy::CurvilinearCosyBuilder::set_tangent));
  m.def(
     "normalize_angle", py::overload_cast<double>(&tam::helpers::geometry::normalize_angle),
     "Normalize angle to ensure it is in the interval of [-pi, pi[")
    .def(
      "get_curvature_from_points",
      py::overload_cast<
        const Eigen::Ref<const Eigen::VectorXd>, const Eigen::Ref<const Eigen::VectorXd>>(
        &tam::helpers::geometry::get_curvature_from_points))
    .def(
      "get_curvature_from_heading",
      py::overload_cast<
        const Eigen::Ref<const Eigen::VectorXd>, const Eigen::Ref<const Eigen::VectorXd>>(
        &tam::helpers::geometry::get_curvature_from_heading));
  py::class_<tam::helpers::euler_rotations::Eigen3D>(m, "Eigen3D")
    .def(py::init())
    .def(py::init<const Eigen::VectorXd &, const Eigen::VectorXd &, const Eigen::VectorXd &>())
    .def_readwrite("x", &tam::helpers::euler_rotations::Eigen3D::x)
    .def_readwrite("y", &tam::helpers::euler_rotations::Eigen3D::y)
    .def_readwrite("z", &tam::helpers::euler_rotations::Eigen3D::z);
  m.def(
    "vector_to_2d", py::overload_cast<
                      const tam::helpers::euler_rotations::Eigen3D &, const Eigen::VectorXd &,
                      const Eigen::VectorXd &>(&tam::helpers::euler_rotations::vector_to_2d));
  m.def(
    "vector_to_2d",
    py::overload_cast<const tam::types::common::Vector3D<double> &, const double, const double>(
      &tam::helpers::euler_rotations::vector_to_2d));
  m.def(
    "angular_velocities_to_2d",
    py::overload_cast<
      const tam::helpers::euler_rotations::Eigen3D &, const Eigen::VectorXd &,
      const Eigen::VectorXd &>(&tam::helpers::euler_rotations::angular_velocities_to_2d));
  m.def(
    "angular_velocities_to_2d",
    py::overload_cast<const tam::types::common::EulerYPR &, const double, const double>(
      &tam::helpers::euler_rotations::angular_velocities_to_2d));
  m.def(
    "rotate_ypr",
    py::overload_cast<
      const tam::types::common::Vector3D<double> &, const tam::types::common::EulerYPR &>(
      &tam::helpers::euler_rotations::math::rotate_ypr));
  m.def(
    "rotate_inv_rpy",
    py::overload_cast<
      const tam::types::common::Vector3D<double> &, const tam::types::common::EulerYPR &>(
      &tam::helpers::euler_rotations::math::rotate_inv_rpy));
  py::class_<tam::helpers::DelayCompensation>(m, "DelayCompensation")
    .def(py::init<int, double>())
    .def("update_vehicle_odometry", &tam::helpers::DelayCompensation::update_vehicle_odometry)
    .def("compensate", &tam::helpers::DelayCompensation::compensate);
  m.def(
    "eval_model",
    py::overload_cast<
      const tam::types::common::Vector3D<double> &, const tam::types::vehicle_params::Aero &>(
      &tam::helpers::aeordynamics::eval_model));
  m.def(
    "dynamic_tire_radius",
    py::overload_cast<
      const double, const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> &>(
      &tam::helpers::vehicle_dynamics::dynamic_tire_radius));
  m.def(
    "slip_angle",
    py::overload_cast<
      const tam::types::control::Odometry &, const double,
      const tam::types::vehicle_params::Dimension &>(&tam::helpers::vehicle_dynamics::slip_angle));
}
