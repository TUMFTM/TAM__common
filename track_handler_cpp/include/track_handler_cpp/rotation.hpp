// Copyright 2023 Simon Hoffmann
#pragma once
#include <math.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <tuple>
#include <vector>

#include "tum_helpers_cpp/constants.hpp"
#include "tum_helpers_cpp/geometry/geometry.hpp"
namespace tam::common::track
{
/**
 * \brief
 *
 * \param theta
 * \param mu
 * \param phi
 * \return
 */
inline Eigen::Matrix3d get_rotation_matrix(const double theta, const double mu, const double phi)
{
  Eigen::Matrix3d rot;
  rot << std::cos(theta) * std::cos(mu),
    std::cos(theta) * std::sin(mu) * std::sin(phi) - std::sin(theta) * std::cos(phi),
    std::cos(theta) * std::sin(mu) * std::cos(phi) + std::sin(theta) * std::sin(phi),
    // row 2
    std::sin(theta) * std::cos(mu),
    std::sin(theta) * std::sin(mu) * std::sin(phi) + std::cos(theta) * std::cos(phi),
    std::sin(theta) * std::sin(mu) * std::cos(phi) - std::cos(theta) * std::sin(phi),
    // row 3
    -std::sin(mu), std::cos(mu) * std::sin(phi), std::cos(mu) * std::cos(phi);

  return rot;
}
/**
 * \brief
 *
 * \param theta
 * \param mu
 * \param phi
 * \return
 */
inline double calc_2d_heading_from_chi(
  const double chi, const double theta, const double mu, const double phi)
{
  Eigen::Vector3d vec_loc = Eigen::Vector3d(std::cos(chi), std::sin(chi), 0.0);
  // Todo (Simon): Normalized required
  Eigen::Vector3d vec_glob = (get_rotation_matrix(theta, mu, phi) * vec_loc).normalized();
  return tam::helpers::geometry::calc_heading(Eigen::Vector2d(vec_glob[0], vec_glob[1]));
}
/**
 * \brief
 *
 * \param theta
 * \param mu
 * \param phi
 * \return
 */
inline Eigen::VectorXd calc_2d_heading_from_chi(
  const Eigen::Ref<const Eigen::VectorXd> chi, const Eigen::Ref<const Eigen::VectorXd> theta,
  const Eigen::Ref<const Eigen::VectorXd> mu, const Eigen::Ref<const Eigen::VectorXd> phi)
{
  Eigen::VectorXd heading(chi.size());  // Todo (Simon): CHeck for size
  for (int i = 0; i < static_cast<int>(chi.size()); ++i) {
    heading[i] = calc_2d_heading_from_chi(chi[i], theta[i], mu[i], phi[i]);  // Todo: Check
  }
  return heading;
}

inline Eigen::Vector3d angles_to_velocity_frame(const double chi, const double theta, const double mu, const double phi){
  Eigen::Vector3d basis_vel_in_road_x (std::cos(chi), std::sin(chi), 0.0); 
  Eigen::Vector3d basis_vel_in_road_y (-std::sin(chi), std::cos(chi), 0.0); 
  Eigen::Vector3d basis_vel_in_road_z (0.0, 0.0, 1.0); 

  // representation of basis vectors (velocity frame) in intertial frame
  Eigen::Vector3d vel_global_x = (get_rotation_matrix(theta, mu, phi) * basis_vel_in_road_x).normalized(); 
  Eigen::Vector3d vel_global_y = (get_rotation_matrix(theta, mu, phi) * basis_vel_in_road_y).normalized(); 
  Eigen::Vector3d vel_global_z = (get_rotation_matrix(theta, mu, phi) * basis_vel_in_road_z).normalized(); 

  Eigen::Matrix3d vel_global; 
  vel_global.col(0) = vel_global_x; 
  vel_global.col(1) = vel_global_y; 
  vel_global.col(2) = vel_global_z; 
  Eigen::Vector3d euler_angle = vel_global.eulerAngles(2, 1, 0); 

  return euler_angle;  
}


inline Eigen::MatrixXd angles_to_velocity_frame (
  const Eigen::Ref<const Eigen::VectorXd> chi, const Eigen::Ref<const Eigen::VectorXd> theta,
  const Eigen::Ref<const Eigen::VectorXd> mu, const Eigen::Ref<const Eigen::VectorXd> phi)
  {
    Eigen::MatrixXd euler_angles(chi.size(), 3); 

    for(int i = 0; i < static_cast<int>(chi.size()); ++i){
      Eigen::Vector3d euler_angle = angles_to_velocity_frame(chi[i], theta[i], mu[i], phi[i]); 
      euler_angles.row(i) = euler_angle; 
    }
    
    return euler_angles; 
  }

/**
 * \brief
 *
 * \param theta
 * \param mu
 * \param phi
 * \return
 */
inline double calc_chi_from_2d_heading(
  const double heading, const double theta, const double mu, const double phi)
{
  Eigen::Vector3d vec_glob = Eigen::Vector3d(std::cos(heading), std::sin(heading), 0.0);
  Eigen::Vector3d vec_loc =
    (get_rotation_matrix(theta, mu, phi).transpose() * vec_glob).normalized();
  return tam::helpers::geometry::calc_heading(Eigen::Vector2d(vec_loc[0], vec_loc[1]));
  // Todo: Check
}
/**
 * \brief
 *
 * \param theta
 * \param mu
 * \param phi
 * \return
 */
inline Eigen::VectorXd calc_chi_from_2d_heading(
  const Eigen::Ref<const Eigen::VectorXd> heading, const Eigen::Ref<const Eigen::VectorXd> theta,
  const Eigen::Ref<const Eigen::VectorXd> mu, const Eigen::Ref<const Eigen::VectorXd> phi)
{
  Eigen::VectorXd chi(heading.size());  // Todo (Simon): CHeck for size
  for (int i = 0; i < static_cast<int>(heading.size()); ++i) {
    chi[i] = calc_chi_from_2d_heading(heading[i], theta[i], mu[i], phi[i]);
  }
  return chi;
}
}  // namespace tam::common::track
