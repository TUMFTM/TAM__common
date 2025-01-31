// Copyright 2024 Maximilian Leitenstern
#pragma once

#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "tum_helpers_cpp/numerical.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_types_cpp/data_per_wheel.hpp"
#include "tum_types_cpp/tire.hpp"
#include "tum_types_cpp/vehicle.hpp"
namespace tam::helpers::vehicle_dynamics
{
/**
 * @brief Definition for dynamic tire radius
 * @param vel Velocity over ground of the tire
 * @param params Parameters of the four tires
 * @return tam::types::common::DataPerWheel<double> Dynamic tire radii
 */
tam::types::common::DataPerWheel<double> dynamic_tire_radius(
  const double vel_x,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & params);
/**
 * @brief Calculate slip ratios of the vehicle
 * @param vel_x Velocity over ground of the vehicle
 * @param wheelspeeds Rotational velocity of the wheels
 * @param params Parameters of the tire
 * @return tam::types::common::DataPerWheel<double>
 */
tam::types::common::DataPerWheel<double> long_slip(
  const tam::types::control::Odometry & odom, const double & steering_angle,
  const tam::types::common::DataPerWheel<double> & wheelspeeds,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & tire_params,
  const tam::types::vehicle_params::Dimension & params);
/**
 * @brief Calculate slip angles of the vehicle
 * @param odom pose of vehicle
 * @param steering_angle steering angle of vehicle
 * @param params vehicle parameters for dimension
 * @return tam::types::common::DataPerWheel<double>
 */
tam::types::common::DataPerWheel<double> slip_angle(
  const tam::types::control::Odometry & odom, const double steering_angle,
  const tam::types::vehicle_params::Dimension & params);
}  // namespace tam::helpers::vehicle_dynamics
