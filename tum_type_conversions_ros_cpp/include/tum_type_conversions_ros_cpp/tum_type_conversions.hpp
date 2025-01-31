// Copyright 2023 Philipp Pitschi
#pragma once
#include <math.h>

// ROS
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>

// messages
#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_path.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tier4_planning_msgs/msg/trajectory.hpp"
#include "tum_msgs/msg/tum_control_constraints.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel.hpp"
// types
#include "tum_helpers_cpp/type_conversion.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_types_cpp/prediction.hpp"
namespace tam::type_conversions
{
// Basic types
builtin_interfaces::msg::Time header_stamp_msg_from_type(const uint64_t & header);
tam::types::common::Vector3D<double> vector_3d_type_from_msg(
  geometry_msgs::msg::Vector3 const & msg);
tam::types::common::Vector3D<double> vector_3d_type_from_msg(geometry_msgs::msg::Point const & msg);
geometry_msgs::msg::Vector3 vector_3d_msg_from_type(
  tam::types::common::Vector3D<double> const & vector_3d);
geometry_msgs::msg::Point point_msg_from_type(
  tam::types::common::Vector3D<double> const & vector_3d);
tam::types::common::DataPerWheel<double> data_per_wheel_type_from_msg(
  tum_msgs::msg::TUMFloat64PerWheel const & msg);
tum_msgs::msg::TUMFloat64PerWheel data_per_wheel_msg_from_type(
  tam::types::common::DataPerWheel<double> const & data);
tam::types::ErrorLvl error_type_from_diagnostic_level(unsigned char lvl);
unsigned char diagnostic_level_from_type(const tam::types::ErrorLvl & lvl);

// State Estimation
tam::types::control::AccelerationwithCovariances accel_with_covariance_stamped_type_from_msg(
  geometry_msgs::msg::AccelWithCovarianceStamped const & msg);
geometry_msgs::msg::AccelWithCovarianceStamped accel_with_covariance_stamped_msg_from_type(
  const tam::types::control::AccelerationwithCovariances & acceleration);
tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  sensor_msgs::msg::Imu const & msg);
tam::types::control::Odometry odometry_type_from_msg(nav_msgs::msg::Odometry const & odometry);
nav_msgs::msg::Odometry odometry_msg_from_type(const tam::types::control::Odometry & odometry);
tam::types::control::Odometry odometry_type_from_imu_msg(sensor_msgs::msg::Imu const & msg);

// Control
tam::types::control::Trajectory trajectory_type_from_msg(
  tier4_planning_msgs::msg::Trajectory const & msg);
tier4_planning_msgs::msg::Trajectory trajectory_msg_from_type(
  const tam::types::control::Trajectory & trajectory);
tam::types::control::ControlConstraints constraint_type_from_msg(
  const tum_msgs::msg::TUMControlConstraints & msg);
tum_msgs::msg::TUMControlConstraints constraint_msg_from_type(
  const tam::types::control::ControlConstraints & constraints);
tam::types::control::ControlConstraintPoint constraint_type_from_msg(
  const tum_msgs::msg::TUMControlConstraintPoint & msg);
tum_msgs::msg::TUMControlConstraintPoint constraint_msg_from_type(
  const tam::types::control::ControlConstraintPoint & constraint);

// Prediction
tam::types::prediction::TrackedObjects tracked_objects_type_from_msg(
  autoware_auto_perception_msgs::msg::TrackedObjects const & msg);
autoware_auto_perception_msgs::msg::TrackedObjects tracked_objects_msg_from_type(
  const tam::types::prediction::TrackedObjects & tracked_objects);
autoware_auto_perception_msgs::msg::PredictedObjects predicted_objects_msg_from_type(
  const tam::types::prediction::PredictedObjects & predicted_objects);
tam::types::prediction::PredictedObjectsWithOwnedTrackedObjects predicted_objects_type_from_msg(
  const autoware_auto_perception_msgs::msg::PredictedObjects & msg);

// STUFF BELOW THIS LINE IS JUST HERE FOR BACKWARDS COMPATIBILITY
// DO NOT USE ANY MORE
// ==============================================================================
tam::types::control::Odometry odometry_type_from_imu_msg(
  const sensor_msgs::msg::Imu::SharedPtr msg);
tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  const sensor_msgs::msg::Imu::SharedPtr msg);
tam::types::common::DataPerWheel<double> data_per_wheel_type_from_msg(
  const tum_msgs::msg::TUMFloat64PerWheel::SharedPtr msg);
}  // namespace tam::type_conversions
namespace tam::type::conversions::cpp
{
/// @brief Here for backwards compability
tam::types::control::Trajectory Trajectory_type_from_msg(
  const tier4_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

/// @brief Here for backwards compability
tam::types::control::Odometry Odometry_type_from_msg(const nav_msgs::msg::Odometry::SharedPtr msg);
}  // namespace tam::type::conversions::cpp
