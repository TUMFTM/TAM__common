// Copyright 2023 Simon Hoffmann
#pragma once
#include <math.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <tuple>
#include <vector>

#include "tum_helpers_cpp/constants.hpp"
namespace tam::common::track
{
/**
 * \brief
 *
 * \param mu
 * \param phi
 * \param chi
 * \param ax
 * \param ay
 * \return
 */
inline std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> calc_a_tilde(
  const Eigen::Ref<const Eigen::MatrixXd> mu, const Eigen::Ref<const Eigen::MatrixXd> phi,
  const Eigen::Ref<const Eigen::MatrixXd> chi, const Eigen::Ref<const Eigen::MatrixXd> ax,
  const Eigen::Ref<const Eigen::MatrixXd> ay)
{
  Eigen::MatrixXd ax_tilde =
    ax.array() +
    tam::constants::g_earth * (-mu.array().sin() * chi.array().cos() +
                               mu.array().cos() * phi.array().sin() * chi.array().sin());
  Eigen::MatrixXd ay_tilde =
    ay.array() +
    tam::constants::g_earth * (mu.array().sin() * chi.array().sin() +
                               mu.array().cos() * phi.array().sin() * chi.array().cos());
  return {ax_tilde, ay_tilde};
}
/**
 * \brief
 *
 * \param mu
 * \param phi
 * \param chi
 * \param ax
 * \param ay
 * \return
 */
inline std::tuple<double, double> calc_a_tilde(
  const double mu, const double phi, const double chi, const double ax, const double ay)
{
  double ax_tilde =
    ax + tam::constants::g_earth *
           (std::sin(-mu) * std::cos(chi) + std::cos(mu) * std::sin(phi) * std::sin(chi));
  double ay_tilde =
    ay + tam::constants::g_earth *
           (std::sin(mu) * std::sin(chi) + std::cos(mu) * std::sin(phi) * std::cos(chi));
  return {ax_tilde, ay_tilde};
}
/**
 * \brief
 *
 * \param V
 * \param n
 * \param chi
 * \param Omega_z
 * \return
 */
inline Eigen::MatrixXd calc_s_dot(
  const Eigen::Ref<const Eigen::MatrixXd> V, const Eigen::Ref<const Eigen::MatrixXd> n,
  const Eigen::Ref<const Eigen::MatrixXd> Omega_z, const Eigen::Ref<const Eigen::MatrixXd> chi)

{
  return (V.array() * chi.array().cos()).array() / (1.0 - n.array() * Omega_z.array()).array();
}
/**
 * \brief
 *
 * \param V
 * \param n
 * \param chi
 * \param Omega_z
 * \return
 */
inline double calc_s_dot(const double V, const double n, const double Omega_z, const double chi)

{
  return (V * std::cos(chi)) / (1.0 - n * Omega_z);
}
/**
 * \brief calc g_tilde in 3D space
 *
 * \param Omega_x
 * \param Omega_y
 * \param mu
 * \param phi
 * \param s_dot
 * \param V
 * \param chi
 * \return g_tilde
 */
inline Eigen::MatrixXd calc_g_tilde(
  const Eigen::Ref<const Eigen::MatrixXd> Omega_x, const Eigen::Ref<const Eigen::MatrixXd> Omega_y,
  const Eigen::Ref<const Eigen::MatrixXd> mu, const Eigen::Ref<const Eigen::MatrixXd> phi,
  const Eigen::Ref<const Eigen::MatrixXd> chi, const Eigen::Ref<const Eigen::MatrixXd> s_dot,
  const Eigen::Ref<const Eigen::MatrixXd> V)  // Todo (Simon): is it hat?
{
  Eigen::MatrixXd g_tilde =
    (Omega_x.array() * chi.array().sin() - Omega_y.array() * chi.array().cos()) * s_dot.array() *
      V.array() +
    tam::constants::g_earth * mu.array().cos() * phi.array().cos();
  return g_tilde.cwiseMax(0);
}
/**
 * \brief calc g_tilde in 3D space
 *
 * \param Omega_x
 * \param Omega_y
 * \param mu
 * \param phi
 * \param s_dot
 * \param V
 * \param chi
 * \return g_tilde
 */
inline double calc_g_tilde(
  const double Omega_x, const double Omega_y, const double mu, const double phi, const double chi,
  const double s_dot,
  const double V)  // Todo (Simon): is it hat?
{
  double g_tilde = (Omega_x * std::sin(chi) - Omega_y * std::cos(chi)) * s_dot * V +
                   tam::constants::g_earth * std::cos(mu) * std::cos(phi);
  return g_tilde;
}
/**
 * \brief calculates a_hat in 3D space
 *
 * \param mu
 * \param phi
 * \param chi
 * \param ax_tilde
 * \param ay_tilde
 * \return tuple of ax_hat and ay_hat
 */
inline std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> calc_a_hat(
  const Eigen::Ref<const Eigen::MatrixXd> mu, const Eigen::Ref<const Eigen::MatrixXd> phi,
  const Eigen::Ref<const Eigen::MatrixXd> chi, const Eigen::Ref<const Eigen::MatrixXd> ax_tilde,
  const Eigen::Ref<const Eigen::MatrixXd> ay_tilde)  // Todo (Simon): is it hat?
{
  Eigen::MatrixXd ax =
    ax_tilde.array() -
    tam::constants::g_earth * (-mu.array().sin() * chi.array().cos() +
                               mu.array().cos() * phi.array().sin() * chi.array().sin());
  Eigen::MatrixXd ay =
    ay_tilde.array() -
    tam::constants::g_earth * (mu.array().sin() * chi.array().sin() +
                               mu.array().cos() * phi.array().sin() * chi.array().cos());
  return {ax, ay};
}
/**
 * \brief calculates a_hat in 3D space
 *
 * \param mu
 * \param phi
 * \param chi
 * \param ax_tilde
 * \param ay_tilde
 * \return tuple of ax_hat and ay_hat
 */
inline std::tuple<double, double> calc_a_hat(
  const double mu, const double phi, const double chi, const double ax_tilde,
  const double ay_tilde)  // Todo (Simon): is it hat?
{
  double ax =
    ax_tilde - tam::constants::g_earth *
                 (std::sin(-mu) * std::cos(chi) + std::cos(mu) * std::sin(phi) * std::sin(chi));
  double ay =
    ay_tilde - tam::constants::g_earth *
                 (std::sin(mu) * std::sin(chi) + std::cos(mu) * std::sin(phi) * std::cos(chi));
  return {ax, ay};
}
}  // namespace tam::common::track
