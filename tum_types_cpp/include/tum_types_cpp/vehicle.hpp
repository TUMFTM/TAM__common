// Copyright 2024 Maximilian Leitenstern
#pragma once

#include <vector>
#include "tum_types_cpp/common.hpp"
namespace tam::types::vehicle_params
{
struct Actuator
{
  double brake_delay{0.0};
  double throttle_delay{0.0};
  double steering_delay{0.0};
  double gear_delay{0.0};
};
struct Aero
{
  double air_density{0.0};
  double drag_coeff{0.0};
  double lift_coeff{0.0};
  double cross_track_area{0.0};
  double diff_cog_x{0.0};
  double diff_cog_y{0.0};
  double diff_cog_z{0.0};
};
struct AeroModelOutput
{
  // Aerodynamic forces acting on the cog of the vehicle
  tam::types::common::Vector3D<double> force_cog{0.0, 0.0, 0.0};
  // Aerodynamic torque on the vehicle body
  tam::types::common::Vector3D<double> torque{0.0, 0.0, 0.0};
};
struct Brake
{
  double brake_bias_front{0.0};
  double disc_area{0.0};
  double disc_radius{0.0};
  double pad_area{0.0};
  double pad_mean_radius{0.0};
  double pad_number{0.0};
  double piston_diameter{0.0};
  double max_pressure{0.0};
  double friction_coeff{0.0};
};
struct Dimension
{
  double track_width_front{0.0};
  double track_width_rear{0.0};
  double wheelbase{0.0};
  double distance_to_front_axle{0.0};
  double cog_height{0.0};
  double length{0.0};
  double width{0.0};
  double height{0.0};
};
struct Drivetrain
{
  double clutch_engagement_start{0.0};
  double clutch_max_torque_slope{0.0};
  double drivetrain_efficiency{0.0};
  double transmission_ratio{0.0};
  std::vector<double> gear_ratios{};
};
struct EngineMap
{
  std::vector<double> rpms{};
  std::vector<double> throttles{};
  std::vector<double> torques{};
};
struct Engine
{
  double rev_idle{0.0};
  double rev_max{0.0};
  EngineMap engine_map;
};
struct Inertia
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double engine{0.0};
  double wheel_front{0.0};
  double wheel_rear{0.0};
};
struct Steering
{
  double min_angle{0.0};
  double max_angle{0.0};
  double max_rate{0.0};
  double ratio{0.0};
};
struct Mass
{
  double total{0.0};
  double wheel_front{0.0};
  double wheel_rear{0.0};
};
struct RollingResistance
{
  double coeff{0.0};
};
struct Vehicle
{
public:
  Actuator actuator;
  Aero aero;
  Brake brake;
  Dimension dimension;
  Drivetrain drivetrain;
  Engine engine;
  Inertia inertia;
  Steering steering;
  Mass mass;
  RollingResistance rolling_resistance;
};
}  // namespace tam::types::vehicle_params

