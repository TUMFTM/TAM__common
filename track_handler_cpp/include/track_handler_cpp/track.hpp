// Copyright 2023 Simon Hoffmann
#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "track_handler_cpp/track_helpers.hpp"
#include "track_handler_cpp/track_types.hpp"
#include "tum_helpers_cpp/common.hpp"
#include "tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp"
#include "tum_helpers_cpp/file_handling.hpp"
#include "tum_helpers_cpp/numerical.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::common
{
using tam::helpers::track::s_mod;
class Track
{
protected:
  explicit Track(TrackData && data);

private:
  tam::helpers::cosy::CurvilinearCosySharedPtr cosy_;
  std::unique_ptr<TrackData> data_;

  void init();
  void create_normal_vector();
  void create_trackbounds();
  void calc_d_omega();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static std::unique_ptr<Track> create_from_csv(
    const std::string & path, TrackReferenceLines ref = TrackReferenceLines::RACELINE);
  size_t length() const { return data_->data.at(TrackData::s).rows(); }
  bool on_track(const double x, const double y, const double margin) const;
  tam::helpers::cosy::CurvilinearCosySharedPtr get_cosy_handle() { return cosy_; }
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> calc_apparent_acceleration(
    const Eigen::Ref<const Eigen::MatrixXd> s, const Eigen::Ref<const Eigen::MatrixXd> n,
    const Eigen::Ref<const Eigen::MatrixXd> chi, const Eigen::Ref<const Eigen::MatrixXd> ax,
    const Eigen::Ref<const Eigen::MatrixXd> ay, const Eigen::Ref<const Eigen::MatrixXd> V) const;
  std::tuple<double, double, double> calc_apparent_acceleration(
    const double s, const double n, const double chi, const double ax, const double ay,
    const double V) const;
  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> calc_acceleration(
    const Eigen::Ref<const Eigen::MatrixXd> s, const Eigen::Ref<const Eigen::MatrixXd> chi,
    const Eigen::Ref<const Eigen::MatrixXd> ax_tilde,
    const Eigen::Ref<const Eigen::MatrixXd> ay_tilde) const;
  std::tuple<double, double> calc_acceleration(
    const double s, const double chi, const double ax_tilde, const double ay_tilde) const;
  Eigen::Vector3d sn2cartesian(const double s, const double n) const;
  Eigen::MatrixX3d sn2cartesian(
    const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> n) const;
  Eigen::Vector3d get_3d_from_2d(const double x, const double y) const;
  Eigen::Vector2d project_2d_point_on_track(const double x, const double y) const;
  Eigen::MatrixX2d project_2d_point_on_track(
    Eigen::Ref<const Eigen::VectorXd> x, Eigen::Ref<const Eigen::VectorXd> y) const;
  double calc_2d_heading_from_chi(const double s, const double chi) const;
  Eigen::VectorXd calc_2d_heading_from_chi(
    const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> chi) const;
  Eigen::Vector3d angles_to_velocity_frame(const double s, const double chi) const; 
  Eigen::MatrixXd angles_to_velocity_frame(
    const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> chi) const;
  double calc_chi_from_2d_heading(const double s, const double heading) const;
  Eigen::VectorXd calc_chi_from_2d_heading(
    const Eigen::Ref<const Eigen::VectorXd> s,
    const Eigen::Ref<const Eigen::VectorXd> heading) const;
  std::tuple <int, double> get_sector(const double s, const int num_sectors) const;
  // Data Access
  const Eigen::Ref<Eigen::VectorXd> s_coord() const { return data_->data.at(TrackData::s); }
  const Eigen::Ref<Eigen::VectorXd> ref_line_x() const { return data_->data.at(TrackData::x); }
  const Eigen::Ref<Eigen::VectorXd> ref_line_y() const { return data_->data.at(TrackData::y); }
  const Eigen::Ref<Eigen::VectorXd> ref_line_z() const { return data_->data.at(TrackData::z); }
  const Eigen::Ref<Eigen::VectorXd> theta() const { return data_->data.at(TrackData::theta); }
  const Eigen::Ref<Eigen::VectorXd> mu() const { return data_->data.at(TrackData::mu); }
  const Eigen::Ref<Eigen::VectorXd> phi() const { return data_->data.at(TrackData::phi); }
  const Eigen::Ref<Eigen::VectorXd> d_theta() const { return data_->data.at(TrackData::dtheta); }
  const Eigen::Ref<Eigen::VectorXd> d_mu() const { return data_->data.at(TrackData::dmu); }
  const Eigen::Ref<Eigen::VectorXd> d_phi() const { return data_->data.at(TrackData::dphi); }
  const Eigen::Ref<Eigen::VectorXd> omega_x() const { return data_->data.at(TrackData::omega_x); }
  const Eigen::Ref<Eigen::VectorXd> omega_y() const { return data_->data.at(TrackData::omega_y); }
  const Eigen::Ref<Eigen::VectorXd> omega_z() const { return data_->data.at(TrackData::omega_z); }
  const Eigen::Ref<Eigen::VectorXd> left_bound_x() const
  {
    return data_->data.at(TrackData::tb_left_x);
  }
  const Eigen::Ref<Eigen::VectorXd> left_bound_y() const
  {
    return data_->data.at(TrackData::tb_left_y);
  }
  const Eigen::Ref<Eigen::VectorXd> left_bound_z() const
  {
    return data_->data.at(TrackData::tb_left_z);
  }
  const Eigen::Ref<Eigen::VectorXd> right_bound_x() const
  {
    return data_->data.at(TrackData::tb_right_x);
  }
  const Eigen::Ref<Eigen::VectorXd> right_bound_y() const
  {
    return data_->data.at(TrackData::tb_right_y);
  }
  const Eigen::Ref<Eigen::VectorXd> right_bound_z() const
  {
    return data_->data.at(TrackData::tb_right_z);
  }
  const Eigen::Ref<Eigen::VectorXd> normal_x() const { return data_->data.at(TrackData::normal_x); }
  const Eigen::Ref<Eigen::VectorXd> normal_y() const { return data_->data.at(TrackData::normal_y); }
  const Eigen::Ref<Eigen::VectorXd> normal_z() const { return data_->data.at(TrackData::normal_z); }
  const Eigen::Ref<Eigen::VectorXd> d_omega_x() const
  {
    return data_->data.at(TrackData::domega_x);
  }
  const Eigen::Ref<Eigen::VectorXd> d_omega_y() const
  {
    return data_->data.at(TrackData::domega_y);
  }
  const Eigen::Ref<Eigen::VectorXd> d_omega_z() const
  {
    return data_->data.at(TrackData::domega_z);
  }
  const Eigen::Ref<Eigen::VectorXd> trackwidth_left() const
  {
    return data_->data.at(TrackData::w_left);
  }
  const Eigen::Ref<Eigen::VectorXd> trackwidth_right() const
  {
    return data_->data.at(TrackData::w_right);
  }
  // Templated getters for interpolated access
  /**
   * @brief Get ref_line_x interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd ref_line_x(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::x));
  }
  std::vector<double> ref_line_x(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::x));
  }
  double ref_line_x(const double & s) const
  {
    // std::fmod()
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::x));
  }
  /**
   * @brief Get ref_line_y interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd ref_line_y(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::y));
  }
  std::vector<double> ref_line_y(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::y));
  }
  double ref_line_y(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::y));
  }
  /**
   * @brief Get ref_line_z interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd ref_line_z(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::z));
  }
  std::vector<double> ref_line_z(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::z));
  }
  double ref_line_z(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::z));
  }
  /**
   * @brief Get theta interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  //
  Eigen::MatrixXd theta(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::theta));
  }
  std::vector<double> theta(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::theta));
  }
  double theta(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::theta));
  }
  /**
   * @brief Get Mu interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd mu(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::mu));
  }
  std::vector<double> mu(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::mu));
  }
  double mu(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::mu));
  }
  /**
   * @brief Get phi interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd phi(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::phi));
  }
  std::vector<double> phi(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::phi));
  }
  double phi(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::phi));
  }
  /**
   * @brief Get d_theta interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd d_theta(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::dtheta));
  }
  std::vector<double> d_theta(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::dtheta));
  }
  double d_theta(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::dtheta));
  }
  /**
   * @brief Get d_mu interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd d_mu(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::dmu));
  }
  std::vector<double> d_mu(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::dmu));
  }
  double d_mu(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::dmu));
  }
  /**
   * @brief Get d_phi interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd d_phi(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::dphi));
  }
  std::vector<double> d_phi(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::dphi));
  }
  double d_phi(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::dphi));
  }
  /**
   * @brief Get omega_x interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd omega_x(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::omega_x));
  }
  std::vector<double> omega_x(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::omega_x));
  }
  double omega_x(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::omega_x));
  }
  /**
   * @brief Get omega_y interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd omega_y(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::omega_y));
  }
  std::vector<double> omega_y(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::omega_y));
  }
  double omega_y(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::omega_y));
  }
  /**
   * @brief Get omega_z interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd omega_z(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::omega_z));
  }
  std::vector<double> omega_z(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::omega_z));
  }
  double omega_z(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::omega_z));
  }
  /**
   * @brief Get left_bound_x interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd left_bound_x(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_left_x));
  }
  std::vector<double> left_bound_x(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_left_x));
  }
  double left_bound_x(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_left_x));
  }
  /**
   * @brief Get left_bound_y interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd left_bound_y(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_left_y));
  }
  std::vector<double> left_bound_y(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_left_y));
  }
  double left_bound_y(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_left_y));
  }
  /**
   * @brief Get left_bound_z interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd left_bound_z(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_left_z));
  }
  std::vector<double> left_bound_z(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_left_z));
  }
  double left_bound_z(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_left_z));
  }
  /**
   * @brief Get reght_bound_x interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd right_bound_x(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_right_x));
  }
  std::vector<double> right_bound_x(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_right_x));
  }
  double right_bound_x(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_right_x));
  }
  /**
   * @brief Get right_bound_y interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd right_bound_y(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_right_y));
  }
  std::vector<double> right_bound_y(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_right_y));
  }
  double right_bound_y(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_right_y));
  }
  /**
   * @brief Get right_bound_z interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd right_bound_z(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_right_z));
  }
  std::vector<double> right_bound_z(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_right_z));
  }
  double right_bound_z(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::tb_right_z));
  }
  /**
   * @brief Get normal_x interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd normal_x(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_x));
  }
  std::vector<double> normal_x(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_x));
  }
  double normal_x(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_x));
  }
  /**
   * @brief Get normal_y interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd normal_y(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_y));
  }
  std::vector<double> normal_y(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_y));
  }
  double normal_y(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_y));
  }
  /**
   * @brief Get normal_z interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd normal_z(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_z));
  }
  std::vector<double> normal_z(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_z));
  }
  double normal_z(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_z));
  }
  /**
   * @brief Get d_omega_x interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd d_omega_x(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::domega_x));
  }
  std::vector<double> d_omega_x(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::domega_x));
  }
  double d_omega_x(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::domega_x));
  }
  /**
   * @brief Get d_omega_y interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd d_omega_y(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::domega_y));
  }
  std::vector<double> d_omega_y(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::domega_y));
  }
  double d_omega_y(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::domega_y));
  }
  /**
   * @brief Get d_omega_z interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd d_omega_z(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::domega_z));
  }
  std::vector<double> d_omega_z(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::domega_z));
  }
  double d_omega_z(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::domega_z));
  }
  /**
   * @brief Get track_width_left interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd trackwidth_left(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::w_left));
  }
  std::vector<double> trackwidth_left(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::w_left));
  }
  double trackwidth_left(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::w_left));
  }
  /**
   * @brief Get track_width_right interpolated with s
   *
   * @param s
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd trackwidth_right(const Eigen::Ref<const Eigen::MatrixXd> s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::w_right));
  }
  std::vector<double> trackwidth_right(const std::vector<double> & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::w_right));
  }
  double trackwidth_right(const double & s) const
  {
    return tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::w_right));
  }
};
}  // namespace tam::common
