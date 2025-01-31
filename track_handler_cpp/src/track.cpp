// Copyright 2023 Simon Hoffmann
#include "track_handler_cpp/track.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "track_handler_cpp/acceleration.hpp"
#include "track_handler_cpp/rotation.hpp"
#include "track_handler_cpp/track_io.hpp"
#include "tum_helpers_cpp/numerical.hpp"
namespace tam::common
{
Track::Track(TrackData && data)
{
  // Init data
  data_ = std::make_unique<TrackData>(std::move(data));
  // Init Track
  this->init();
}
void Track::init()
{
  // Todo: set heading & s from hear
  this->cosy_ =
    tam::helpers::cosy::CurvilinearCosy::create(
      data_->data.at(TrackData::x), data_->data.at(TrackData::y), data_->data.at(TrackData::z))
      ->build();
  create_normal_vector();
  create_trackbounds();
  calc_d_omega();
  if (!tam::helpers::track::has_equal_sized_data(data_->data)){
      throw std::invalid_argument(
        "[track_handler_cpp]: Trying to load track content with unequal size");
  }
}
std::unique_ptr<Track> Track::create_from_csv(const std::string & path, TrackReferenceLines ref)
{  // make unique doesn't allow protected access
  return std::unique_ptr<Track>(new Track(tam::common::get_track_from_file(path, ref)));
}
void Track::create_normal_vector()
{
  const auto theta = this->theta().array();
  const auto mu = this->mu().array();
  const auto phi = this->phi().array();
  Eigen::MatrixXd normal;
  normal.resize(this->s_coord().rows(), 3);
  normal.col(0) = theta.cos() * mu.sin() * phi.sin() - theta.sin() * phi.cos();
  normal.col(1) = theta.sin() * mu.sin() * phi.sin() + theta.cos() * phi.cos();
  normal.col(2) = mu.cos() * phi.sin();
  normal.rowwise().normalize();
  this->data_->data.at(TrackData::normal_x) = normal.col(0);
  this->data_->data.at(TrackData::normal_y) = normal.col(1);
  this->data_->data.at(TrackData::normal_z) = normal.col(2);
}
void Track::create_trackbounds()
{
  data_->data.at(TrackData::tb_left_x) =
    data_->data.at(TrackData::x).array() +
    data_->data.at(TrackData::normal_x).array() * data_->data.at(TrackData::w_left).array();
  data_->data.at(TrackData::tb_left_y) =
    data_->data.at(TrackData::y).array() +
    data_->data.at(TrackData::normal_y).array() * data_->data.at(TrackData::w_left).array();
  data_->data.at(TrackData::tb_left_z) =
    data_->data.at(TrackData::z).array() +
    data_->data.at(TrackData::normal_z).array() * data_->data.at(TrackData::w_left).array();
  //
  data_->data.at(TrackData::tb_right_x) =
    data_->data.at(TrackData::x).array() +
    data_->data.at(TrackData::normal_x).array() * data_->data.at(TrackData::w_right).array();
  data_->data.at(TrackData::tb_right_y) =
    data_->data.at(TrackData::y).array() +
    data_->data.at(TrackData::normal_y).array() * data_->data.at(TrackData::w_right).array();
  data_->data.at(TrackData::tb_right_z) =
    data_->data.at(TrackData::z).array() +
    data_->data.at(TrackData::normal_z).array() * data_->data.at(TrackData::w_right).array();
}
void Track::calc_d_omega()
{
  data_->data.at(TrackData::domega_x) =
    tam::helpers::numerical::gradient(this->omega_x(), this->s_coord());
  data_->data.at(TrackData::domega_y) =
    tam::helpers::numerical::gradient(this->omega_y(), this->s_coord());
  data_->data.at(TrackData::domega_z) =
    tam::helpers::numerical::gradient(this->omega_z(), this->s_coord());
}
bool Track::on_track(const double x, const double y, const double margin = 0.0) const
{
  auto sn = cosy_->convert_to_sn_and_get_idx(x, y);
  double w_left =
    tam::helpers::numerical::interp_from_idx(this->trackwidth_left(), std::get<1>(sn));
  double w_right =
    tam::helpers::numerical::interp_from_idx(this->trackwidth_right(), std::get<1>(sn));
  return (std::get<0>(sn)[1] >= w_right - margin) && (std::get<0>(sn)[1] <= w_left + margin);
}
std::tuple<double, double, double> Track::calc_apparent_acceleration(
  const double s, const double n, const double chi, const double ax, const double ay,
  const double V) const
{
  auto [ax_tilde, ay_tilde] = tam::common::track::calc_a_tilde(mu(s), phi(s), chi, ax, ay);
  double s_dot = tam::common::track::calc_s_dot(V, n, omega_z(s), chi);
  auto g_tilde =
    tam::common::track::calc_g_tilde(omega_x(s), omega_y(s), mu(s), phi(s), chi, s_dot, V);
  return {ax_tilde, ay_tilde, g_tilde};
}
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Track::calc_apparent_acceleration(
  const Eigen::Ref<const Eigen::MatrixXd> s, const Eigen::Ref<const Eigen::MatrixXd> n,
  const Eigen::Ref<const Eigen::MatrixXd> chi, const Eigen::Ref<const Eigen::MatrixXd> ax,
  const Eigen::Ref<const Eigen::MatrixXd> ay, const Eigen::Ref<const Eigen::MatrixXd> V) const
{
  auto [ax_tilde, ay_tilde] = tam::common::track::calc_a_tilde(mu(s), phi(s), chi, ax, ay);
  Eigen::MatrixXd s_dot = tam::common::track::calc_s_dot(V, n, omega_z(s), chi);
  auto g_tilde =
    tam::common::track::calc_g_tilde(omega_x(s), omega_y(s), mu(s), phi(s), chi, s_dot, V);
  return {ax_tilde, ay_tilde, g_tilde};
}
std::tuple<double, double> Track::calc_acceleration(
  const double s, const double chi, const double ax_tilde, const double ay_tilde) const
{
  return tam::common::track::calc_a_hat(mu(s), phi(s), chi, ax_tilde, ay_tilde);
}
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> Track::calc_acceleration(
  const Eigen::Ref<const Eigen::MatrixXd> s, const Eigen::Ref<const Eigen::MatrixXd> chi,
  const Eigen::Ref<const Eigen::MatrixXd> ax_tilde,
  const Eigen::Ref<const Eigen::MatrixXd> ay_tilde) const
{
  return tam::common::track::calc_a_hat(mu(s), phi(s), chi, ax_tilde, ay_tilde);
}
Eigen::Vector3d Track::sn2cartesian(const double s, const double n) const
{
  Eigen::Vector3d norm;
  Eigen::Vector3d ref_p;
  for (int i = 0; i < 3; ++i) {
    norm(i) = tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_x + i));
    ref_p(i) = tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::x + i));
  }
  return ref_p.array() + (norm.array() * n).array();
}
Eigen::MatrixX3d Track::sn2cartesian(
  const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> n) const
{
  Eigen::MatrixX3d norm(s.size(), 3);
  Eigen::MatrixX3d ref_p(s.size(), 3);

  for (int i = 0; i < 3; ++i) {
    norm.col(i) = tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_x + i));
    ref_p.col(i) = tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::x + i));
  }
  return ref_p.array() + norm.array().colwise() * n.array();
}
double Track::calc_2d_heading_from_chi(const double s, const double chi) const
{
  return tam::common::track::calc_2d_heading_from_chi(chi, theta(s), mu(s), phi(s));
}
Eigen::VectorXd Track::calc_2d_heading_from_chi(
  const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> chi) const
{
  return tam::common::track::calc_2d_heading_from_chi(chi, theta(s), mu(s), phi(s));
}
Eigen::Vector3d Track::angles_to_velocity_frame(const double s, const double chi) const
{
  return tam::common::track::angles_to_velocity_frame(chi, theta(s), mu(s), phi(s));
}
Eigen::MatrixXd Track::angles_to_velocity_frame(
  const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> chi) const
{
  return tam::common::track::angles_to_velocity_frame(chi, theta(s), mu(s), phi(s));
}
double Track::calc_chi_from_2d_heading(const double s, const double heading) const
{
  return tam::common::track::calc_chi_from_2d_heading(heading, theta(s), mu(s), phi(s));
}
Eigen::VectorXd Track::calc_chi_from_2d_heading(
  const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> heading) const
{
  return tam::common::track::calc_chi_from_2d_heading(heading, theta(s), mu(s), phi(s));
}
Eigen::Vector2d Track::project_2d_point_on_track(const double x, const double y) const
{
  return cosy_->convert_to_sn(x, y);
}
Eigen::MatrixX2d Track::project_2d_point_on_track(
  Eigen::Ref<const Eigen::VectorXd> x, Eigen::Ref<const Eigen::VectorXd> y) const
{
  Eigen::MatrixX2d sn_out(x.rows(), 2);
  for (int i = 0; i < x.size(); ++i) {
    sn_out.row(i) = cosy_->convert_to_sn(x(i), y(i)).transpose();
  }
  return sn_out;
}
Eigen::Vector3d Track::get_3d_from_2d(const double x, const double y) const
{
  auto sn = project_2d_point_on_track(x, y);

  return sn2cartesian(sn.x(), sn.y());
}

std::tuple <int, double> Track::get_sector(const double s, const int num_sectors) const
{
  // Return if Num segments is less than 1
  if (num_sectors <= 0) {
    return std::make_tuple(-1, -1.0);
  }

  // Base caluclations
  double s_max = data_->data.at(TrackData::s)(Eigen::last);
  double s_curr = s_mod(s, s_max);
  double sector_length = s_max / num_sectors;

  // Sector calculations
  double sector_num = std::floor(s_curr / sector_length);
  double remainder = s_curr - (sector_num * sector_length);

  return std::make_tuple(
    static_cast<int>(sector_num),
    remainder
  );
}

}  // namespace tam::common
