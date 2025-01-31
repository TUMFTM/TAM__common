// Copyright 2023 Simon Sagmeister
#include <string>
namespace CoordinateFrames
{
// ========================================================================================
// Global Frames
// ========================================================================================
// Geodetic frame in lat long
constexpr std::string_view geodetic = "geodetic";
// Local cartesian frame that is created using a a reference position in lat long.
// Transformation from geodetic to local cartesian can't be done with tf2
// Orientation of the local cartesian frame: X -> East, Y -> North, Z -> Up
// Heading Zero points towards the x axis. Orientation angles are defined in the interval [-pi,pi[
constexpr std::string_view local_cartesian = "local_cartesian";
// Birds eye view of the local cartesian frame
constexpr std::string_view local_cartesian_footprint = "local_cartesian_footprint";
// ========================================================================================
// Curvilinear Frames
// ========================================================================================
// Frenet coordinate system spanning around the centerline
// Not transformable with tf2
constexpr std::string_view centerline_curvilinear = "centerline_curvilinear";
// Frenet coordinate system spanning around the raceline
// Not transformable with tf2
constexpr std::string_view raceline_curvilinear = "raceline_curvilinear";
// Frenet coordinate system spanning around the local trajectory
// Not transformable with tf2
constexpr std::string_view local_trajectory_curvilinear = "local_trajectory_curvilinear";
// ========================================================================================
// Vehicle frames
// ========================================================================================
// Coordiante system originated in the vehicle's center of gravity.
// X: in longitdunal direction of the vehicle
// Y: in lateral direction of the vehicle
// Z: up
constexpr std::string_view vehicle_cg = "vehicle_cg";
// Corresponding birds eye view
constexpr std::string_view vehicle_cg_footprint = "vehicle_cg_footprint";
// Coordiante system originated in the vehicle's rear axle middle
// X: in longitdunal direction of the vehicle
// Y: in lateral direction of the vehicle
// Z: up
constexpr std::string_view vehicle_rear_axle_middle = "vehicle_rear_axle_middle";
// Coordiante system below the vehicle's rear axle middle on the ground plane
// X: in longitdunal direction of the vehicle
// Y: in lateral direction of the vehicle
// Z: up
constexpr std::string_view vehicle_rear_axle_middle_ground = "vehicle_rear_axle_middle_ground";
// Coordiante system originated in the vehicle's center of gravity.
// X: in the direction of the current velocity vector
// Y: perpendicular to the current velocity vector
// Z: up
constexpr std::string_view vehicle_velocity_cg = "vehicle_velocity_cg";
// Corresponding birds eye view coordinate system
constexpr std::string_view vehicle_velocity_cg_footprint = "vehicle_velocity_cg_footprint";
// Same as vehicle_cg, but in the vehicle pose at the trajectory point instead of the current pose
constexpr std::string_view vehicle_cg_at_trajectory_point = "vehicle_cg_at_trajectory_point";
// Same as vehicle_cg_footprint, but in the vehicle pose at the trajectory point instead of the
// current pose
constexpr std::string_view vehicle_cg_footprint_at_trajectory_point =
  "vehicle_cg_footprint_at_trajectory_point";
// Same as vehicle_velocity_cg, but in the vehicle pose at the trajectory point instead of the
// current pose
constexpr std::string_view vehicle_velocity_cg_at_trajectory_point =
  "vehicle_velocity_cg_at_trajectory_point";
// Same as vehicle_velocity_cg_footprint, but in the vehicle pose at the trajectory point instead of
// the current pose
constexpr std::string_view vehicle_velocity_cg_footprint_at_trajectory_point =
  "vehicle_velocity_cg_footprint_at_trajectory_point";
// ========================================================================================
// Sensor frames - The transformations for these should be defined in the URDF
// These are included with name here to avoid using a wrong transform accidentally (e.g. loading
// wrong urdf)
// ========================================================================================
constexpr std::string_view novatel_top_gps = "novatel_top_gps";
constexpr std::string_view novatel_top_imu = "novatel_top_imu";
constexpr std::string_view novatel_bottom_gps = "novatel_bottom_gps";
constexpr std::string_view novatel_bottom_imu = "novatel_bottom_imu";
constexpr std::string_view vectornav_gps = "vectornav_gps";
constexpr std::string_view vectornav_imu = "vectornav_imu";
constexpr std::string_view luminar_front = "luminar_front";
constexpr std::string_view luminar_left = "luminar_left";
constexpr std::string_view luminar_right = "luminar_right";
constexpr std::string_view camera_front_left = "camera_front_left";
constexpr std::string_view camera_front_right = "camera_front_right";
constexpr std::string_view camera_rear_left = "camera_rear_left";
constexpr std::string_view camera_rear_right = "camera_rear_right";
constexpr std::string_view camera_front_1 = "camera_front_1";
constexpr std::string_view camera_front_2 = "camera_front_2";
constexpr std::string_view radar_front = "radar_front";
constexpr std::string_view radar_left = "radar_left";
constexpr std::string_view radar_right = "radar_right";
constexpr std::string_view radar_rear = "radar_rear";
}  // namespace CoordinateFrames
