#include <eigen3/Eigen/Dense>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

// #include "tum_helpers_cpp/geometry/geometry.hpp"

#include "track_handler_cpp/race_track_handler.hpp"
#include "track_handler_cpp/raceline.hpp"
#include "track_handler_cpp/track.hpp"

// #include <pybind11/stl.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Vector3.h>
// // include "pybind11_test/test_struct.hpp"

namespace py = pybind11;
PYBIND11_MODULE(_cpp_binding, m)
{
  py::enum_<TrackReferenceLines>(m, "TrackReferenceLines")
    .value("RACELINE", TrackReferenceLines::RACELINE)
    .value("CENTERLINE", TrackReferenceLines::CENTERLINE);
  py::class_<tam::common::RaceTrackHandler>(m, "RaceTrackHandler")
    .def("from_pkg_config", &tam::common::RaceTrackHandler::from_pkg_config, "")
    .def("create_track", &tam::common::RaceTrackHandler::create_track, "")
    .def("create_raceline_track", &tam::common::RaceTrackHandler::create_raceline_track, "")
    .def("create_centerline_track", &tam::common::RaceTrackHandler::create_centerline_track, "")
    .def("create_pitlane", &tam::common::RaceTrackHandler::create_pitlane, "")
    .def("create_raceline", &tam::common::RaceTrackHandler::create_raceline, "")
    .def("create_track_prediction", &tam::common::RaceTrackHandler::create_track_prediction, "")
    .def("create_raceline_track_prediction", &tam::common::RaceTrackHandler::create_raceline_track_prediction, "")
    .def("create_centerline_track_prediction", &tam::common::RaceTrackHandler::create_centerline_track_prediction, "")
    .def("create_pitlane_prediction", &tam::common::RaceTrackHandler::create_pitlane_prediction, "")
    .def(
      "create_raceline_prediction", &tam::common::RaceTrackHandler::create_raceline_prediction, "")
    .def("get_track_name", &tam::common::RaceTrackHandler::get_track_name, "")
    .def("get_track_file", &tam::common::RaceTrackHandler::get_track_file, "")
    .def("return_raceline_path", &tam::common::RaceTrackHandler::return_raceline_path, "")
    .def("get_pit_file", &tam::common::RaceTrackHandler::get_pit_file, "")
    .def("return_pitlane_path", &tam::common::RaceTrackHandler::return_pitlane_path, "")
    .def("get_raceline_file", &tam::common::RaceTrackHandler::get_raceline_file, "")
    .def("get_track_file_pred", &tam::common::RaceTrackHandler::get_track_file_pred, "")
    .def("get_pit_file_pred", &tam::common::RaceTrackHandler::get_pit_file_pred, "")
    .def("get_raceline_file_pred", &tam::common::RaceTrackHandler::get_raceline_file_pred, "")
    .def("get_initial_heading", &tam::common::RaceTrackHandler::get_initial_heading, "")
    .def("get_geo_origin", &tam::common::RaceTrackHandler::get_geo_origin, "")
    .def("get_sim_start_pos", &tam::common::RaceTrackHandler::get_sim_start_pos, "")
    .def("get_track_path_default", &tam::common::RaceTrackHandler::get_track_path_default, "")
    .def("get_param", &tam::common::RaceTrackHandler::get_param, "")
    .def(
      "get_raceline_path_default", &tam::common::RaceTrackHandler::get_raceline_path_default, "");

  py::class_<tam::common::Raceline>(m, "Raceline")
    .def("create_from_csv", &tam::common::Raceline::create_from_csv)
    .def("s", &tam::common::Raceline::s)
    .def("v", [](tam::common::Raceline & rl) { return rl.v(); })
    .def("v", [](tam::common::Raceline & rl, const double & a) { return rl.v(a); })
    .def(
      "v",
      [](tam::common::Raceline & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.v(a);
      })
    .def("n", [](tam::common::Raceline & rl) { return rl.n(); })
    .def("n", [](tam::common::Raceline & rl, const double & a) { return rl.n(a); })
    .def(
      "n",
      [](tam::common::Raceline & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.n(a);
      })
    .def("chi", [](tam::common::Raceline & rl) { return rl.chi(); })
    .def("chi", [](tam::common::Raceline & rl, const double & a) { return rl.chi(a); })
    .def(
      "chi",
      [](tam::common::Raceline & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.chi(a);
      })
    .def("ax", [](tam::common::Raceline & rl) { return rl.ax(); })
    .def("ax", [](tam::common::Raceline & rl, const double & a) { return rl.ax(a); })
    .def(
      "ax",
      [](tam::common::Raceline & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.ax(a);
      })
    .def("ay", [](tam::common::Raceline & rl) { return rl.ay(); })
    .def("ay", [](tam::common::Raceline & rl, const double & a) { return rl.ay(a); })
    .def(
      "ay",
      [](tam::common::Raceline & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.ay(a);
      })
    .def("jx", [](tam::common::Raceline & rl) { return rl.jx(); })
    .def("jx", [](tam::common::Raceline & rl, const double & a) { return rl.jx(a); })
    .def(
      "jx",
      [](tam::common::Raceline & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.jx(a);
      })
    .def("jy", [](tam::common::Raceline & rl) { return rl.jy(); })
    .def("jy", [](tam::common::Raceline & rl, const double & a) { return rl.jy(a); })
    .def("jy", [](tam::common::Raceline & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
      return rl.jy(a);
    });

  py::class_<tam::common::Track>(m, "Track")
    .def("create_from_csv", &tam::common::Track::create_from_csv, py::arg("ref") = TrackReferenceLines::RACELINE)
    .def("length", &tam::common::Track::length)
    .def("on_track", &tam::common::Track::on_track)
    .def("get_cosy_handle", &tam::common::Track::get_cosy_handle)

    .def(
      "calc_apparent_acceleration",
      py::overload_cast<
        const double, const double, const double, const double, const double, const double>(
        &tam::common::Track::calc_apparent_acceleration, py::const_))
    .def(
      "calc_apparent_acceleration",
      py::overload_cast<
        Eigen::Ref<const Eigen::MatrixXd>, Eigen::Ref<const Eigen::MatrixXd>,
        Eigen::Ref<const Eigen::MatrixXd>, Eigen::Ref<const Eigen::MatrixXd>,
        Eigen::Ref<const Eigen::MatrixXd>, Eigen::Ref<const Eigen::MatrixXd>>(
        &tam::common::Track::calc_apparent_acceleration, py::const_))

    .def(
      "calc_acceleration",
      py::overload_cast<const double, const double, const double, const double>(
        &tam::common::Track::calc_acceleration, py::const_))
    .def(
      "calc_acceleration",
      py::overload_cast<
        Eigen::Ref<const Eigen::MatrixXd>, Eigen::Ref<const Eigen::MatrixXd>,
        Eigen::Ref<const Eigen::MatrixXd>, Eigen::Ref<const Eigen::MatrixXd>>(
        &tam::common::Track::calc_acceleration, py::const_))

    .def(
      "sn2cartesian",
      py::overload_cast<const double, const double>(&tam::common::Track::sn2cartesian, py::const_))
    .def(
      "sn2cartesian",
      py::overload_cast<Eigen::Ref<const Eigen::VectorXd>, Eigen::Ref<const Eigen::VectorXd>>(
        &tam::common::Track::sn2cartesian, py::const_))
    .def(
      "project_2d_point_on_track", py::overload_cast<const double, const double>(
                                     &tam::common::Track::project_2d_point_on_track, py::const_))
    .def(
      "project_2d_point_on_track",
      py::overload_cast<Eigen::Ref<const Eigen::VectorXd>, Eigen::Ref<const Eigen::VectorXd>>(
        &tam::common::Track::project_2d_point_on_track, py::const_))

    .def(
      "calc_2d_heading_from_chi", py::overload_cast<const double, const double>(
                                    &tam::common::Track::calc_2d_heading_from_chi, py::const_))
    .def(
      "calc_2d_heading_from_chi",
      py::overload_cast<
        const Eigen::Ref<const Eigen::VectorXd>, const Eigen::Ref<const Eigen::VectorXd>>(
        &tam::common::Track::calc_2d_heading_from_chi, py::const_))
    .def(
      "angles_to_velocity_frame", py::overload_cast<const double, const double>(
                                    &tam::common::Track::angles_to_velocity_frame, py::const_))
    .def(
       "angles_to_velocity_frame",
       py::overload_cast<
         const Eigen::Ref<const Eigen::VectorXd>, const Eigen::Ref<const Eigen::VectorXd>>(
         &tam::common::Track::angles_to_velocity_frame, py::const_))
    .def(
      "calc_chi_from_2d_heading", py::overload_cast<const double, const double>(
                                    &tam::common::Track::calc_chi_from_2d_heading, py::const_))
    .def(
      "calc_chi_from_2d_heading",
      py::overload_cast<
        const Eigen::Ref<const Eigen::VectorXd>, const Eigen::Ref<const Eigen::VectorXd>>(
        &tam::common::Track::calc_chi_from_2d_heading, py::const_))

    .def("get_sector", &tam::common::Track::get_sector, "")
    .def("s_coord", &tam::common::Track::s_coord, "")
    .def("ref_line_x", [](tam::common::Track & rl) { return rl.ref_line_x(); })
    .def("ref_line_x", [](tam::common::Track & rl, const double & a) { return rl.ref_line_x(a); })
    .def(
      "ref_line_x",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.ref_line_x(a);
      })
    .def("ref_line_y", [](tam::common::Track & rl) { return rl.ref_line_y(); })
    .def("ref_line_y", [](tam::common::Track & rl, const double & a) { return rl.ref_line_y(a); })
    .def(
      "ref_line_y",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.ref_line_y(a);
      })

    .def("ref_line_z", [](tam::common::Track & rl) { return rl.ref_line_z(); })
    .def("ref_line_z", [](tam::common::Track & rl, const double & a) { return rl.ref_line_z(a); })
    .def(
      "ref_line_z",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.ref_line_z(a);
      })
    .def("theta", [](tam::common::Track & rl) { return rl.theta(); })
    .def("theta", [](tam::common::Track & rl, const double & a) { return rl.theta(a); })
    .def(
      "theta",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.theta(a);
      })

    .def("mu", [](tam::common::Track & rl) { return rl.mu(); })
    .def("mu", [](tam::common::Track & rl, const double & a) { return rl.mu(a); })
    .def(
      "mu",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.mu(a);
      })

    .def("phi", [](tam::common::Track & rl) { return rl.phi(); })
    .def("phi", [](tam::common::Track & rl, const double & a) { return rl.phi(a); })
    .def(
      "phi",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.phi(a);
      })

    .def("d_theta", [](tam::common::Track & rl) { return rl.d_theta(); })
    .def("d_theta", [](tam::common::Track & rl, const double & a) { return rl.d_theta(a); })
    .def(
      "d_theta",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.d_theta(a);
      })

    .def("d_mu", [](tam::common::Track & rl) { return rl.d_mu(); })
    .def("d_mu", [](tam::common::Track & rl, const double & a) { return rl.d_mu(a); })
    .def(
      "d_mu",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.d_mu(a);
      })

    .def("d_phi", [](tam::common::Track & rl) { return rl.d_phi(); })
    .def("d_phi", [](tam::common::Track & rl, const double & a) { return rl.d_phi(a); })
    .def(
      "d_phi",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.d_phi(a);
      })

    .def("omega_x", [](tam::common::Track & rl) { return rl.omega_x(); })
    .def("omega_x", [](tam::common::Track & rl, const double & a) { return rl.omega_x(a); })
    .def(
      "omega_x",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.omega_x(a);
      })

    .def("omega_y", [](tam::common::Track & rl) { return rl.omega_y(); })
    .def("omega_y", [](tam::common::Track & rl, const double & a) { return rl.omega_y(a); })
    .def(
      "omega_y",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.omega_y(a);
      })

    .def("omega_z", [](tam::common::Track & rl) { return rl.omega_z(); })
    .def("omega_z", [](tam::common::Track & rl, const double & a) { return rl.omega_z(a); })
    .def(
      "omega_z",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.omega_z(a);
      })

    .def("left_bound_x", [](tam::common::Track & rl) { return rl.left_bound_x(); })
    .def(
      "left_bound_x", [](tam::common::Track & rl, const double & a) { return rl.left_bound_x(a); })
    .def(
      "left_bound_x",
      [](
        tam::common::Track & rl,
        const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.left_bound_x(a); })

    .def("left_bound_y", [](tam::common::Track & rl) { return rl.left_bound_y(); })
    .def(
      "left_bound_y", [](tam::common::Track & rl, const double & a) { return rl.left_bound_y(a); })
    .def(
      "left_bound_y",
      [](
        tam::common::Track & rl,
        const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.left_bound_y(a); })

    .def("left_bound_z", [](tam::common::Track & rl) { return rl.left_bound_z(); })
    .def(
      "left_bound_z", [](tam::common::Track & rl, const double & a) { return rl.left_bound_z(a); })
    .def(
      "left_bound_z",
      [](
        tam::common::Track & rl,
        const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.left_bound_z(a); })

    .def("right_bound_x", [](tam::common::Track & rl) { return rl.right_bound_x(); })
    .def(
      "right_bound_x",
      [](tam::common::Track & rl, const double & a) { return rl.right_bound_x(a); })
    .def(
      "right_bound_x",
      [](
        tam::common::Track & rl,
        const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.right_bound_x(a); })

    .def("right_bound_y", [](tam::common::Track & rl) { return rl.right_bound_y(); })
    .def(
      "right_bound_y",
      [](tam::common::Track & rl, const double & a) { return rl.right_bound_y(a); })
    .def(
      "right_bound_y",
      [](
        tam::common::Track & rl,
        const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.right_bound_y(a); })

    .def("right_bound_z", [](tam::common::Track & rl) { return rl.right_bound_z(); })
    .def(
      "right_bound_z",
      [](tam::common::Track & rl, const double & a) { return rl.right_bound_z(a); })
    .def(
      "right_bound_z",
      [](
        tam::common::Track & rl,
        const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.right_bound_z(a); })

    .def("normal_x", [](tam::common::Track & rl) { return rl.normal_x(); })
    .def("normal_x", [](tam::common::Track & rl, const double & a) { return rl.normal_x(a); })
    .def(
      "normal_x", [](
                    tam::common::Track & rl,
                    const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.normal_x(a); })

    .def("normal_y", [](tam::common::Track & rl) { return rl.normal_y(); })
    .def("normal_y", [](tam::common::Track & rl, const double & a) { return rl.normal_y(a); })
    .def(
      "normal_y", [](
                    tam::common::Track & rl,
                    const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.normal_y(a); })

    .def("normal_z", [](tam::common::Track & rl) { return rl.normal_z(); })
    .def("normal_z", [](tam::common::Track & rl, const double & a) { return rl.normal_z(a); })
    .def(
      "normal_z", [](
                    tam::common::Track & rl,
                    const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.normal_z(a); })

    .def("d_omega_x", [](tam::common::Track & rl) { return rl.d_omega_x(); })
    .def("d_omega_x", [](tam::common::Track & rl, const double & a) { return rl.d_omega_x(a); })
    .def(
      "d_omega_x", [](
                     tam::common::Track & rl,
                     const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.d_omega_x(a); })

    .def("d_omega_y", [](tam::common::Track & rl) { return rl.d_omega_y(); })
    .def("d_omega_y", [](tam::common::Track & rl, const double & a) { return rl.d_omega_y(a); })
    .def(
      "d_omega_y", [](
                     tam::common::Track & rl,
                     const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.d_omega_y(a); })

    .def("d_omega_z", [](tam::common::Track & rl) { return rl.d_omega_z(); })
    .def("d_omega_z", [](tam::common::Track & rl, const double & a) { return rl.d_omega_z(a); })
    .def(
      "d_omega_z", [](
                     tam::common::Track & rl,
                     const py::EigenDRef<const Eigen::MatrixXd> & a) { return rl.d_omega_z(a); })

    .def("trackwidth_left", [](tam::common::Track & rl) { return rl.trackwidth_left(); })
    .def(
      "trackwidth_left",
      [](tam::common::Track & rl, const double & a) { return rl.trackwidth_left(a); })
    .def(
      "trackwidth_left",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.trackwidth_left(a);
      })

    .def("trackwidth_right", [](tam::common::Track & rl) { return rl.trackwidth_right(); })
    .def(
      "trackwidth_right",
      [](tam::common::Track & rl, const double & a) { return rl.trackwidth_right(a); })
    .def(
      "trackwidth_right",
      [](tam::common::Track & rl, const py::EigenDRef<const Eigen::MatrixXd> & a) {
        return rl.trackwidth_right(a);
      });
}
