// Copyright 2023 Philip Pitschi
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"

#include "tum_type_conversions_ros_cpp/orientation.hpp"
namespace tam::type_conversions
{
builtin_interfaces::msg::Time header_stamp_msg_from_type(const uint64_t & header_stamp)
{
  builtin_interfaces::msg::Time msg;
  msg.sec = floor(header_stamp / 1e9);
  msg.nanosec = header_stamp % (int)1e9;
  return msg;
}
tam::types::control::Trajectory trajectory_type_from_msg(
  tier4_planning_msgs::msg::Trajectory const & msg)
{
  tam::types::control::Trajectory input;
  input.points.reserve(msg.points.size());
  for (std::size_t i = 0; i < msg.points.size(); i++) {
    tam::types::control::TrajectoryPoint point_;
    point_.position_m.x = msg.points[i].pose.position.x;  // x
    point_.position_m.y = msg.points[i].pose.position.y;  // y
    point_.position_m.z = msg.points[i].pose.position.z;  // not needed
    point_.orientation_rad = tam::helpers::types::to_vector3d(
      tam::types::conversion::quaternion_msg_to_euler_type(msg.points[i].pose.orientation));
    point_.velocity_mps.x = msg.points[i].twist.linear.x;                  // vx
    point_.velocity_mps.y = msg.points[i].twist.linear.y;                  // not needed
    point_.velocity_mps.z = msg.points[i].twist.linear.z;                  // not needed
    point_.acceleration_mps2.x = msg.points[i].accel.linear.x;             // ax
    point_.acceleration_mps2.y = msg.points[i].accel.linear.y;             // not needed
    point_.acceleration_mps2.z = msg.points[i].accel.linear.z;             // not needed
    point_.angular_velocity_radps.x = msg.points[i].twist.angular.x;       // not needed
    point_.angular_velocity_radps.y = msg.points[i].twist.angular.y;       // not needed
    point_.angular_velocity_radps.z = msg.points[i].twist.angular.z;       // not needed
    point_.angular_acceleration_radps2.x = msg.points[i].accel.angular.x;  // not needed
    point_.angular_acceleration_radps2.y = msg.points[i].accel.angular.y;  // not needed
    point_.angular_acceleration_radps2.z = msg.points[i].accel.angular.z;  // not needed
    input.points.emplace_back(point_);
  }
  return input;
}
tier4_planning_msgs::msg::Trajectory trajectory_msg_from_type(
  const tam::types::control::Trajectory & trajectory)
{
  tier4_planning_msgs::msg::Trajectory msg;
  msg.points.reserve(trajectory.points.size());
  for (const auto & point_ : trajectory.points) {
    tier4_planning_msgs::msg::TrajectoryPoint msg_point;
    msg_point.pose.position.x = point_.position_m.x;
    msg_point.pose.position.y = point_.position_m.y;
    msg_point.pose.position.z = point_.position_m.z;
    msg_point.pose.orientation =
      tam::types::conversion::euler_type_to_quaternion_msg(tam::types::common::EulerYPR(
        point_.orientation_rad.z, point_.orientation_rad.y, point_.orientation_rad.x));
    msg_point.twist.linear.x = point_.velocity_mps.x;
    msg_point.twist.linear.y = point_.velocity_mps.y;
    msg_point.twist.linear.z = point_.velocity_mps.z;
    msg_point.accel.linear.x = point_.acceleration_mps2.x;
    msg_point.accel.linear.y = point_.acceleration_mps2.y;
    msg_point.accel.linear.z = point_.acceleration_mps2.z;
    msg_point.twist.angular.x = point_.angular_velocity_radps.x;
    msg_point.twist.angular.y = point_.angular_velocity_radps.y;
    msg_point.twist.angular.z = point_.angular_velocity_radps.z;
    msg_point.accel.angular.x = point_.angular_acceleration_radps2.x;
    msg_point.accel.angular.y = point_.angular_acceleration_radps2.y;
    msg_point.accel.angular.z = point_.angular_acceleration_radps2.z;
    msg.points.emplace_back(msg_point);
  }
  return msg;
}
unsigned char diagnostic_level_from_type(const tam::types::ErrorLvl & lvl)
{
  unsigned char lvl_out;
  switch (lvl) {
    case tam::types::ErrorLvl::OK:
      lvl_out = diagnostic_msgs::msg::DiagnosticStatus::OK;
      break;
    case tam::types::ErrorLvl::STALE:
      lvl_out = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      break;
    case tam::types::ErrorLvl::ERROR:
      lvl_out = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      break;
    case tam::types::ErrorLvl::WARN:
      lvl_out = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;
    default:
      lvl_out = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      std::cerr << "Unknown Error Level \n";
      break;
  }
  return lvl_out;
}
tam::types::ErrorLvl error_type_from_diagnostic_level(unsigned char lvl)
{
  tam::types::ErrorLvl lvl_out;
  switch (lvl) {
    case diagnostic_msgs::msg::DiagnosticStatus::OK:
      lvl_out = tam::types::ErrorLvl::OK;
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::WARN:
      lvl_out = tam::types::ErrorLvl::WARN;
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      lvl_out = tam::types::ErrorLvl::ERROR;
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::STALE:
      lvl_out = tam::types::ErrorLvl::STALE;
      break;
    default:
      lvl_out = tam::types::ErrorLvl::ERROR;
      std::cerr << "Unknown Error Level \n";
      break;
  }
  return lvl_out;
}
tam::types::control::Odometry odometry_type_from_imu_msg(sensor_msgs::msg::Imu const & msg)
{
  tam::types::control::Odometry odometry;
  odometry.angular_velocity_radps.x = msg.angular_velocity.x;
  odometry.angular_velocity_radps.y = msg.angular_velocity.y;
  odometry.angular_velocity_radps.z = msg.angular_velocity.z;

  return odometry;
}
tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  sensor_msgs::msg::Imu const & msg)
{
  tam::types::control::AccelerationwithCovariances acceleration;
  acceleration.acceleration_mps2.x = msg.linear_acceleration.x;
  acceleration.acceleration_mps2.y = msg.linear_acceleration.y;
  acceleration.acceleration_mps2.z = msg.linear_acceleration.z;

  return acceleration;
}
tam::types::control::Odometry odometry_type_from_msg(nav_msgs::msg::Odometry const & msg)
{
  tam::types::control::Odometry odometry;
  odometry.position_m.x = msg.pose.pose.position.x;
  odometry.position_m.y = msg.pose.pose.position.y;
  odometry.position_m.z = msg.pose.pose.position.z;
  odometry.orientation_rad = tam::helpers::types::to_vector3d(
    tam::types::conversion::quaternion_msg_to_euler_type(msg.pose.pose.orientation));
  odometry.pose_covariance = msg.pose.covariance;
  odometry.angular_velocity_radps.x = msg.twist.twist.angular.x;
  odometry.angular_velocity_radps.y = msg.twist.twist.angular.y;
  odometry.angular_velocity_radps.z = msg.twist.twist.angular.z;
  odometry.velocity_mps.x = msg.twist.twist.linear.x;
  odometry.velocity_mps.y = msg.twist.twist.linear.y;
  odometry.velocity_mps.z = msg.twist.twist.linear.z;
  odometry.velocity_covariance = msg.twist.covariance;

  return odometry;
}
nav_msgs::msg::Odometry odometry_msg_from_type(const tam::types::control::Odometry & odometry)
{
  nav_msgs::msg::Odometry odometry_msg;
  odometry_msg.pose.pose.position.x = odometry.position_m.x;
  odometry_msg.pose.pose.position.y = odometry.position_m.y;
  odometry_msg.pose.pose.position.z = odometry.position_m.z;
  odometry_msg.pose.pose.orientation =
    tam::types::conversion::euler_type_to_quaternion_msg(tam::types::common::EulerYPR(
      odometry.orientation_rad.z, odometry.orientation_rad.y, odometry.orientation_rad.x));
  odometry_msg.pose.covariance = odometry.pose_covariance;
  odometry_msg.twist.twist.linear.x = odometry.velocity_mps.x;
  odometry_msg.twist.twist.linear.y = odometry.velocity_mps.y;
  odometry_msg.twist.twist.linear.z = odometry.velocity_mps.z;
  odometry_msg.twist.twist.angular.x = odometry.angular_velocity_radps.x;
  odometry_msg.twist.twist.angular.y = odometry.angular_velocity_radps.y;
  odometry_msg.twist.twist.angular.z = odometry.angular_velocity_radps.z;
  odometry_msg.twist.covariance = odometry.velocity_covariance;

  return odometry_msg;
}
geometry_msgs::msg::AccelWithCovarianceStamped accel_with_covariance_stamped_msg_from_type(
  const tam::types::control::AccelerationwithCovariances & acceleration)
{
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration_msg;
  acceleration_msg.accel.accel.linear = vector_3d_msg_from_type(acceleration.acceleration_mps2);
  acceleration_msg.accel.accel.angular =
    vector_3d_msg_from_type(acceleration.angular_acceleration_radps2);
  acceleration_msg.accel.covariance = acceleration.acceleration_covariance;
  return acceleration_msg;
}
tam::types::control::AccelerationwithCovariances accel_with_covariance_stamped_type_from_msg(
  geometry_msgs::msg::AccelWithCovarianceStamped const & msg)
{
  tam::types::control::AccelerationwithCovariances type_;
  type_.acceleration_mps2 = vector_3d_type_from_msg(msg.accel.accel.linear);
  type_.angular_acceleration_radps2 = vector_3d_type_from_msg(msg.accel.accel.angular);
  type_.acceleration_covariance = msg.accel.covariance;
  return type_;
}
geometry_msgs::msg::Vector3 vector_3d_msg_from_type(
  tam::types::common::Vector3D<double> const & vector_3d)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = vector_3d.x;
  msg.y = vector_3d.y;
  msg.z = vector_3d.z;
  return msg;
}
geometry_msgs::msg::Point point_msg_from_type(
  tam::types::common::Vector3D<double> const & vector_3d)
{
  geometry_msgs::msg::Point msg;
  msg.x = vector_3d.x;
  msg.y = vector_3d.y;
  msg.z = vector_3d.z;
  return msg;
}
tam::types::common::Vector3D<double> vector_3d_type_from_msg(
  geometry_msgs::msg::Vector3 const & msg)
{
  return tam::types::common::Vector3D<double>{msg.x, msg.y, msg.z};
}
tam::types::common::Vector3D<double> vector_3d_type_from_msg(geometry_msgs::msg::Point const & msg)
{
  return tam::types::common::Vector3D<double>{msg.x, msg.y, msg.z};
}
tam::types::control::ControlConstraintPoint constraint_type_from_msg(
  const tum_msgs::msg::TUMControlConstraintPoint & msg)
{
  tam::types::control::ControlConstraintPoint pt_out;
  pt_out.a_x_max_mps2.x = msg.ax_max_mps2.x;
  pt_out.a_x_max_mps2.y = msg.ax_max_mps2.y;
  pt_out.a_y_max_mps2.x = msg.ay_max_mps2.x;
  pt_out.a_y_max_mps2.y = msg.ay_max_mps2.y;
  pt_out.a_x_min_mps2.x = msg.ax_min_mps2.x;
  pt_out.a_x_min_mps2.y = msg.ax_min_mps2.y;
  pt_out.a_y_min_mps2.x = msg.ay_min_mps2.x;
  pt_out.a_y_min_mps2.y = msg.ay_min_mps2.y;
  pt_out.a_x_max_engine_mps2 = msg.ax_max_engine_mps2;
  pt_out.shape_factor = msg.shape_factor;
  pt_out.lateral_error_max_m = msg.lateral_error_max_m;
  pt_out.lateral_error_min_m = msg.lateral_error_min_m;
  return pt_out;
}
tum_msgs::msg::TUMControlConstraintPoint constraint_msg_from_type(
  const tam::types::control::ControlConstraintPoint & constraint)
{
  tum_msgs::msg::TUMControlConstraintPoint msg;
  msg.ax_max_mps2.x = constraint.a_x_max_mps2.x;
  msg.ax_max_mps2.y = constraint.a_x_max_mps2.y;
  msg.ay_max_mps2.x = constraint.a_y_max_mps2.x;
  msg.ay_max_mps2.y = constraint.a_y_max_mps2.y;
  msg.ax_min_mps2.x = constraint.a_x_min_mps2.x;
  msg.ax_min_mps2.y = constraint.a_x_min_mps2.y;
  msg.ay_min_mps2.x = constraint.a_y_min_mps2.x;
  msg.ay_min_mps2.y = constraint.a_y_min_mps2.y;
  msg.ax_max_engine_mps2 = constraint.a_x_max_engine_mps2;
  msg.shape_factor = constraint.shape_factor;
  msg.lateral_error_max_m = constraint.lateral_error_max_m;
  msg.lateral_error_min_m = constraint.lateral_error_min_m;
  return msg;
}
tam::types::control::ControlConstraints constraint_type_from_msg(
  const tum_msgs::msg::TUMControlConstraints & msg)
{
  tam::types::control::ControlConstraints out;
  for (const auto & pt : msg.points) {
    out.points.push_back(constraint_type_from_msg(pt));
  }
  out.header.frame_id = msg.header.frame_id;
  out.header.time_stamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec;
  return out;
}
tum_msgs::msg::TUMControlConstraints constraint_msg_from_type(
  const tam::types::control::ControlConstraints & constraints)
{
  tum_msgs::msg::TUMControlConstraints msg;
  msg.points.reserve(constraints.points.size());
  for (const auto & pt : constraints.points) {
    msg.points.push_back(constraint_msg_from_type(pt));
  }
  msg.header.frame_id = constraints.header.frame_id;
  msg.header.stamp = header_stamp_msg_from_type(constraints.header.time_stamp_ns);
  return msg;
}
tum_msgs::msg::TUMFloat64PerWheel data_per_wheel_msg_from_type(
  tam::types::common::DataPerWheel<double> const & data)
{
  tum_msgs::msg::TUMFloat64PerWheel msg;
  msg.front_left = data.front_left;
  msg.front_right = data.front_right;
  msg.rear_left = data.rear_left;
  msg.rear_right = data.rear_right;
  return msg;
}
tam::types::common::DataPerWheel<double> data_per_wheel_type_from_msg(
  tum_msgs::msg::TUMFloat64PerWheel const & msg)
{
  return tam::types::common::DataPerWheel<double>{
    msg.front_left, msg.front_right, msg.rear_left, msg.rear_right};
}
tam::types::prediction::TrackedObjects tracked_objects_type_from_msg(
  autoware_auto_perception_msgs::msg::TrackedObjects const & msg)
{
  tam::types::prediction::TrackedObjects type_{};
  type_.header.time_stamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec;
  type_.header.frame_id = msg.header.frame_id;
  type_.objects.reserve(msg.objects.size());
  for (std::size_t i = 0; i < msg.objects.size(); i++) {
    tam::types::prediction::TrackedObject object_{};
    object_.object_id = msg.objects.at(i).object_id.uuid.at(0);  // SPAX uuid
    object_.is_stationary = msg.objects.at(i).kinematics.is_stationary;
    object_.existence_probability = msg.objects.at(i).existence_probability;
    object_.orientation_rad =
      tam::helpers::types::to_vector3d(tam::types::conversion::quaternion_msg_to_euler_type(
        msg.objects.at(i).kinematics.pose_with_covariance.pose.orientation));
    object_.position_m =
      vector_3d_type_from_msg(msg.objects.at(i).kinematics.pose_with_covariance.pose.position);
    object_.angular_velocity_radps =
      vector_3d_type_from_msg(msg.objects.at(i).kinematics.twist_with_covariance.twist.angular);
    object_.velocity_mps =
      vector_3d_type_from_msg(msg.objects.at(i).kinematics.twist_with_covariance.twist.linear);
    object_.angular_acceleration_radps2 = vector_3d_type_from_msg(
      msg.objects.at(i).kinematics.acceleration_with_covariance.accel.angular);
    object_.acceleration_mps2 = vector_3d_type_from_msg(
      msg.objects.at(i).kinematics.acceleration_with_covariance.accel.linear);
    object_.pose_covariance = msg.objects.at(i).kinematics.pose_with_covariance.covariance;
    object_.velocity_covariance = msg.objects.at(i).kinematics.twist_with_covariance.covariance;
    object_.acceleration_covariance =
      msg.objects.at(i).kinematics.acceleration_with_covariance.covariance;
    type_.objects.emplace_back(object_);
  }
  return type_;
}
autoware_auto_perception_msgs::msg::TrackedObjects tracked_objects_msg_from_type(
  const tam::types::prediction::TrackedObjects & tracked_objects)
{
  autoware_auto_perception_msgs::msg::TrackedObjects msg_{};
  msg_.objects.reserve(tracked_objects.objects.size());
  msg_.header.stamp = header_stamp_msg_from_type(tracked_objects.header.time_stamp_ns);
  msg_.header.frame_id = tracked_objects.header.frame_id;
  for (std::size_t i = 0; i < tracked_objects.objects.size(); i++) {
    autoware_auto_perception_msgs::msg::TrackedObject msg_obj_{};
    msg_obj_.kinematics.pose_with_covariance.pose.orientation =
      tam::types::conversion::euler_type_to_quaternion_msg(tam::types::common::EulerYPR(
        tracked_objects.objects.at(i).orientation_rad.z,
        tracked_objects.objects.at(i).orientation_rad.y,
        tracked_objects.objects.at(i).orientation_rad.x));
    msg_obj_.kinematics.pose_with_covariance.pose.position =
      point_msg_from_type(tracked_objects.objects.at(i).position_m);
    msg_obj_.kinematics.twist_with_covariance.twist.angular =
      vector_3d_msg_from_type(tracked_objects.objects.at(i).angular_velocity_radps);
    msg_obj_.kinematics.twist_with_covariance.twist.linear =
      vector_3d_msg_from_type(tracked_objects.objects.at(i).velocity_mps);
    msg_obj_.kinematics.acceleration_with_covariance.accel.angular =
      vector_3d_msg_from_type(tracked_objects.objects.at(i).angular_acceleration_radps2);
    msg_obj_.kinematics.acceleration_with_covariance.accel.linear =
      vector_3d_msg_from_type(tracked_objects.objects.at(i).acceleration_mps2);
    msg_obj_.kinematics.pose_with_covariance.covariance =
      tracked_objects.objects.at(i).pose_covariance;
    msg_obj_.kinematics.twist_with_covariance.covariance =
      tracked_objects.objects.at(i).velocity_covariance;
    msg_obj_.kinematics.acceleration_with_covariance.covariance =
      tracked_objects.objects.at(i).acceleration_covariance;
    msg_obj_.object_id.uuid.at(0) = tracked_objects.objects.at(i).object_id;
    msg_obj_.kinematics.is_stationary = tracked_objects.objects.at(i).is_stationary;
    msg_obj_.existence_probability = tracked_objects.objects.at(i).existence_probability;
    msg_.objects.emplace_back(msg_obj_);
  }
  return msg_;
}
autoware_auto_perception_msgs::msg::PredictedObjects predicted_objects_msg_from_type(
  const tam::types::prediction::PredictedObjects & pred_objs)
{
  autoware_auto_perception_msgs::msg::PredictedObjects msg_{};
  msg_.objects.reserve(pred_objs.objects.size());
  msg_.header.stamp = header_stamp_msg_from_type(pred_objs.header.time_stamp_ns);
  msg_.header.frame_id = pred_objs.header.frame_id;
  for (std::size_t i = 0; i < pred_objs.objects.size(); i++) {
    autoware_auto_perception_msgs::msg::PredictedObject msg_obj_{};
    msg_obj_.kinematics.predicted_paths.reserve(pred_objs.objects.at(i).predicted_paths.size());
    msg_obj_.kinematics.initial_pose_with_covariance.pose.orientation =
      tam::types::conversion::euler_type_to_quaternion_msg(tam::types::common::EulerYPR(
        pred_objs.objects.at(i).tracked_object.get().orientation_rad.z,
        pred_objs.objects.at(i).tracked_object.get().orientation_rad.y,
        pred_objs.objects.at(i).tracked_object.get().orientation_rad.x));
    msg_obj_.kinematics.initial_pose_with_covariance.pose.position =
      point_msg_from_type(pred_objs.objects.at(i).tracked_object.get().position_m);
    msg_obj_.kinematics.initial_twist_with_covariance.twist.angular =
      vector_3d_msg_from_type(pred_objs.objects.at(i).tracked_object.get().angular_velocity_radps);
    msg_obj_.kinematics.initial_twist_with_covariance.twist.linear =
      vector_3d_msg_from_type(pred_objs.objects.at(i).tracked_object.get().velocity_mps);
    msg_obj_.kinematics.initial_acceleration_with_covariance.accel.angular =
      vector_3d_msg_from_type(
        pred_objs.objects.at(i).tracked_object.get().angular_acceleration_radps2);
    msg_obj_.kinematics.initial_acceleration_with_covariance.accel.linear =
      vector_3d_msg_from_type(pred_objs.objects.at(i).tracked_object.get().acceleration_mps2);
    msg_obj_.kinematics.initial_pose_with_covariance.covariance =
      pred_objs.objects.at(i).tracked_object.get().pose_covariance;
    msg_obj_.kinematics.initial_twist_with_covariance.covariance =
      pred_objs.objects.at(i).tracked_object.get().velocity_covariance;
    msg_obj_.kinematics.initial_acceleration_with_covariance.covariance =
      pred_objs.objects.at(i).tracked_object.get().acceleration_covariance;
    msg_obj_.object_id.uuid.at(0) = pred_objs.objects.at(i).tracked_object.get().object_id;
    msg_obj_.existence_probability =
      pred_objs.objects.at(i).tracked_object.get().existence_probability;
    for (std::size_t j = 0; j < pred_objs.objects.at(i).predicted_paths.size(); j++) {
      autoware_auto_perception_msgs::msg::PredictedPath msg_paths_{};
      msg_paths_.path.reserve(pred_objs.objects.at(i).predicted_paths.at(j).path.size());
      msg_paths_.confidence = pred_objs.objects.at(i).predicted_paths.at(j).confidence;
      float time_step_temp = pred_objs.objects.at(i).predicted_paths.at(j).time_step_s;
      msg_paths_.time_step.sec = floor(time_step_temp);
      msg_paths_.time_step.nanosec = int(time_step_temp * 1e9) % (int)1e9;
      for (std::size_t k = 0; k < pred_objs.objects.at(i).predicted_paths.at(j).path.size(); k++) {
        geometry_msgs::msg::Pose msg_pose_{};
        msg_pose_.position =
          point_msg_from_type(pred_objs.objects.at(i).predicted_paths.at(j).path.at(k).position_m);
        msg_pose_.orientation =
          tam::types::conversion::euler_type_to_quaternion_msg(tam::types::common::EulerYPR(
            pred_objs.objects.at(i).predicted_paths.at(j).path.at(k).orientation_rad.z,
            pred_objs.objects.at(i).predicted_paths.at(j).path.at(k).orientation_rad.y,
            pred_objs.objects.at(i).predicted_paths.at(j).path.at(k).orientation_rad.x));
        msg_paths_.path.emplace_back(msg_pose_);
      }
      msg_obj_.kinematics.predicted_paths.emplace_back(msg_paths_);
    }
    msg_.objects.emplace_back(msg_obj_);
  }
  return msg_;
}
tam::types::prediction::PredictedObjectsWithOwnedTrackedObjects predicted_objects_type_from_msg(
  const autoware_auto_perception_msgs::msg::PredictedObjects & msg)
{
  tam::types::prediction::PredictedObjectsWithOwnedTrackedObjects pred_objs{};
  pred_objs.header.time_stamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec;
  pred_objs.header.frame_id = msg.header.frame_id;
  pred_objs.objects.reserve(msg.objects.size());
  for (const auto & msg_obj : msg.objects) {
    tam::types::prediction::PredictedObjectWithOwnedTrackedObject pred_obj{};
    pred_obj.tracked_object.orientation_rad =
      tam::helpers::types::to_vector3d(tam::types::conversion::quaternion_msg_to_euler_type(
        msg_obj.kinematics.initial_pose_with_covariance.pose.orientation));
    pred_obj.tracked_object.position_m =
      vector_3d_type_from_msg(msg_obj.kinematics.initial_pose_with_covariance.pose.position);
    pred_obj.tracked_object.angular_velocity_radps =
      vector_3d_type_from_msg(msg_obj.kinematics.initial_twist_with_covariance.twist.angular);
    pred_obj.tracked_object.velocity_mps =
      vector_3d_type_from_msg(msg_obj.kinematics.initial_twist_with_covariance.twist.linear);
    pred_obj.tracked_object.angular_acceleration_radps2 = vector_3d_type_from_msg(
      msg_obj.kinematics.initial_acceleration_with_covariance.accel.angular);
    pred_obj.tracked_object.acceleration_mps2 =
      vector_3d_type_from_msg(msg_obj.kinematics.initial_acceleration_with_covariance.accel.linear);
    pred_obj.tracked_object.pose_covariance =
      msg_obj.kinematics.initial_pose_with_covariance.covariance;
    pred_obj.tracked_object.velocity_covariance =
      msg_obj.kinematics.initial_twist_with_covariance.covariance;
    pred_obj.tracked_object.acceleration_covariance =
      msg_obj.kinematics.initial_acceleration_with_covariance.covariance;
    pred_obj.tracked_object.object_id = msg_obj.object_id.uuid.at(0);
    pred_obj.tracked_object.existence_probability = msg_obj.existence_probability;
    pred_obj.predicted_paths.reserve(msg_obj.kinematics.predicted_paths.size());
    for (const auto & msg_path : msg_obj.kinematics.predicted_paths) {
      tam::types::prediction::PredictedPath pred_path{};
      pred_path.confidence = msg_path.confidence;
      float time_step_temp = msg_path.time_step.sec + msg_path.time_step.nanosec / 1e9;
      pred_path.time_step_s = time_step_temp;
      pred_path.path.reserve(msg_path.path.size());
      for (const auto & msg_pose : msg_path.path) {
        tam::types::prediction::Pose pred_pose{};
        pred_pose.position_m = vector_3d_type_from_msg(msg_pose.position);
        pred_pose.orientation_rad = tam::helpers::types::to_vector3d(
          tam::types::conversion::quaternion_msg_to_euler_type(msg_pose.orientation));
        pred_path.path.emplace_back(pred_pose);
      }
      pred_obj.predicted_paths.emplace_back(pred_path);
    }
    pred_objs.objects.emplace_back(pred_obj);
  }
  return pred_objs;
}
tam::types::common::DataPerWheel<double> data_per_wheel_type_from_msg(
  const tum_msgs::msg::TUMFloat64PerWheel::SharedPtr msg)
{
  return data_per_wheel_type_from_msg(*msg);
}
tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  return acceleration_with_covariances_type_from_imu_msg(*msg);
}
tam::types::control::Odometry odometry_type_from_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  return odometry_type_from_imu_msg(*msg);
}
}  // namespace tam::type_conversions
// namespace tam::type_conversions
namespace tam::type::conversions::cpp
{
tam::types::control::Trajectory Trajectory_type_from_msg(
  const tier4_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  return tam::type_conversions::trajectory_type_from_msg(*msg);
}
tam::types::control::Odometry Odometry_type_from_msg(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  return tam::type_conversions::odometry_type_from_msg(*msg);
}
}  // namespace tam::type::conversions::cpp
