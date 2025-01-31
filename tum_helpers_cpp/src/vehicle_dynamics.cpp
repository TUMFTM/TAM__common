// Copyright 2024 Maximilian Leitenstern

#include "tum_helpers_cpp/vehicle_dynamics.hpp"
namespace tam::helpers::vehicle_dynamics
{
/******************************************************
 * @brief Static helper functions for vehicle dynamics
 ******************************************************/
/**
 * @brief Definition for longitudinal slip
 * @param rot_vel Rotational velocity of the wheel
 * @param vel_over_ground Velocity over ground of tire
 * @return double slip ratio
 * @note -> slip ratio is clamped between -1 and 1
 * @note -> slip ratio is calculated as (rot_vel - vel_over_ground) / max(3.0, vel_over_ground)
 * @note -> i.e. vel over ground is bounded by 3.0 m/s in the denominator to avoid numverical issues
 */
static double slip_def(const double rot_vel, const double vel_over_ground)
{
  return std::clamp((rot_vel - vel_over_ground) / std::max(3.0, vel_over_ground), -1.0, 1.0);
}
/**
 * @brief Definition for lateral slip angle of tire
 * @param vx Velocity in x direction
 * @param vy Velocity in y direction
 * @return double slip angle
 * @note -> slip angle is calculated as -atan2(vy, max(1.0, vx))
 * @note -> vx bounded by 1.0 to avoid numerical issues
 */
static double slip_angle_def(const double vx, const double vy)
{
  return -std::atan2(vy, std::max(1.0, vx));
}
/**
 * @brief Definition for dynamic tire radius
 * @param vel Velocity over ground of the tire
 * @param params radius related parameters of the tire
 * @return double Dynamic tire radius (interpolated)
 * @note -> dynamic tire radius interpolated based on lookup table
 */
static double dynamic_tire_radius(
  const double vel_x, const tam::types::tire_params::Radius & params)
{
  return params.radius_20mps *
         tam::helpers::numerical::interp(
           vel_x, params.velocity_scaling.velocity, params.velocity_scaling.factor);
}
/**
 * @brief Calculate velocity vectors of wheels from COG pose in vehicle frame
 * @param odom pose of vehicle
 * @param params vehicle parameters for dimension
 * @return tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>>
 */
static tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>>
velocity_wheel_over_ground(
  const tam::types::control::Odometry & odom, const tam::types::vehicle_params::Dimension & params)
{
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel({0.0, 0.0});
  vel.front_left = {
    odom.velocity_mps.x - params.track_width_front * odom.angular_velocity_radps.z / 2,
    odom.velocity_mps.y + params.distance_to_front_axle * odom.angular_velocity_radps.z};
  vel.front_right = {
    odom.velocity_mps.x + params.track_width_front * odom.angular_velocity_radps.z / 2,
    odom.velocity_mps.y + params.distance_to_front_axle * odom.angular_velocity_radps.z};
  vel.rear_left = {
    odom.velocity_mps.x - params.track_width_rear * odom.angular_velocity_radps.z / 2,
    odom.velocity_mps.y -
      (params.wheelbase - params.distance_to_front_axle) * odom.angular_velocity_radps.z};
  vel.rear_right = {
    odom.velocity_mps.x + params.track_width_rear * odom.angular_velocity_radps.z / 2,
    odom.velocity_mps.y -
      (params.wheelbase - params.distance_to_front_axle) * odom.angular_velocity_radps.z};
  return vel;
}
/**
 * @brief Calculate velocity vectors of wheels from vehicle coordinate frame to tire frame
 * @param steering_angle Steering angle of the vehicle
 * @param velocity_over_ground_vehicle_frame velocity of tires in vehicle frame
 * @return tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>>
 */
static tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>>
velocity_wheel_over_ground_tire_frame(
  const double steering_angle,
  const tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> &
    velocity_over_ground_vehicle_frame)
{
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel({0.0, 0.0});
  Eigen::Matrix2d rot_matrix{
    {std::cos(steering_angle), std::sin(steering_angle)},
    {-std::sin(steering_angle), std::cos(steering_angle)}};

  // Rotate the wheel velocity over ground into the front vehicles coordinate frame
  Eigen::Vector2d fl = rot_matrix * Eigen::Vector2d{
                                      velocity_over_ground_vehicle_frame.front_left.x,
                                      velocity_over_ground_vehicle_frame.front_left.y};
  vel.front_left = {fl.x(), fl.y()};
  Eigen::Vector2d fr = rot_matrix * Eigen::Vector2d{
                                      velocity_over_ground_vehicle_frame.front_right.x,
                                      velocity_over_ground_vehicle_frame.front_right.y};
  vel.front_right = {fr.x(), fr.y()};
  // Nothing for the rea wheel, since they align with the vehicles coordinate system
  // regarding orientation
  vel.rear_left = velocity_over_ground_vehicle_frame.rear_left;
  vel.rear_right = velocity_over_ground_vehicle_frame.rear_right;
  return vel;
}
/**
 * @brief Calculate the rotational velocity of the tire
 * @param vel_x Velocity over ground of the tire
 * @param wheelspeeds Rotational velocity of the wheels
 * @param params Parameters of the tire
 * @return tam::types::common::DataPerWheel<double>
 */
static tam::types::common::DataPerWheel<double> rot_velocity_tire(
  const double vel_x, const tam::types::common::DataPerWheel<double> & wheelspeeds,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & params)
{
  return wheelspeeds * dynamic_tire_radius(vel_x, params);
}
/******************************************************
 * @brief Public implementations for header functions
 ******************************************************/
tam::types::common::DataPerWheel<double> dynamic_tire_radius(
  const double vel_x,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & params)
{
  return tam::types::common::DataPerWheel<double>(
    dynamic_tire_radius(vel_x, params.front_left.radius),
    dynamic_tire_radius(vel_x, params.front_right.radius),
    dynamic_tire_radius(vel_x, params.rear_left.radius),
    dynamic_tire_radius(vel_x, params.rear_right.radius));
}
tam::types::common::DataPerWheel<double> long_slip(
  const tam::types::control::Odometry & odom, const double & steering_angle,
  const tam::types::common::DataPerWheel<double> & wheelspeeds,
  const tam::types::common::DataPerWheel<tam::types::tire_params::Tire> & tire_params,
  const tam::types::vehicle_params::Dimension & params)
{
  // Calculate wheel velocities from COG pose in vehicle frame
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel_wheel =
    velocity_wheel_over_ground(odom, params);
  // Convert vehice frame to tire frame using steering angle
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel_tire =
    velocity_wheel_over_ground_tire_frame(steering_angle, vel_wheel);

  tam::types::common::DataPerWheel<double> rot_vel_tire =
    rot_velocity_tire(odom.velocity_mps.x, wheelspeeds, tire_params);

  tam::types::common::DataPerWheel<double> slip;
  slip.front_left = slip_def(rot_vel_tire.front_left, vel_tire.front_left.x);
  slip.front_right = slip_def(rot_vel_tire.front_right, vel_tire.front_right.x);
  slip.rear_left = slip_def(rot_vel_tire.rear_left, vel_tire.rear_left.x);
  slip.rear_right = slip_def(rot_vel_tire.rear_right, vel_tire.rear_right.x);

  return slip;
}
tam::types::common::DataPerWheel<double> slip_angle(
  const tam::types::control::Odometry & odom, const double steering_angle,
  const tam::types::vehicle_params::Dimension & params)
{
  // Calculate wheel velocities from COG pose in vehicle frame
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel_wheel =
    velocity_wheel_over_ground(odom, params);
  // Convert vehice frame to tire frame using steering angle
  tam::types::common::DataPerWheel<tam::types::common::Vector2D<double>> vel_tire =
    velocity_wheel_over_ground_tire_frame(steering_angle, vel_wheel);
  // Calculate slip angles
  tam::types::common::DataPerWheel<double> slip;
  slip.front_left = slip_angle_def(vel_tire.front_left.x, vel_tire.front_left.y);
  slip.front_right = slip_angle_def(vel_tire.front_right.x, vel_tire.front_right.y);
  slip.rear_left = slip_angle_def(vel_tire.rear_left.x, vel_tire.rear_left.y);
  slip.rear_right = slip_angle_def(vel_tire.rear_right.x, vel_tire.rear_right.y);
  return slip;
}
}  // namespace tam::helpers::vehicle_dynamics
