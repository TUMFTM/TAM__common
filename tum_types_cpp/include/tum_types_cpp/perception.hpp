// Copyright 2024 Maximilian Leitenstern

#pragma once
// https://pcl.readthedocs.io/projects/tutorials/en/latest/adding_custom_ptype.html#how-to-add-a-new-pointt-type
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace tam::types::perception
{
/********************************************************
 * @brief XYZITPoint
 * @brief define custom point type containing timestamp
 ********************************************************/
struct EIGEN_ALIGN16 XYZITPoint  // enforce SSE padding for correct memory alignment
{
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  float intensity;
  double timestamp;
  PCL_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
};
/********************************************************
 * @brief XYZITAEDPoint
 * @brief define custom point type containing timestamp and additional lidar data
 ********************************************************/
struct EIGEN_ALIGN16 XYZITAEDPoint
{
  PCL_ADD_POINT4D;  // quad-word XYZ
  double timestamp;
  float intensity;
  float azimuth;
  float elevation;
  float distance;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
/********************************************************
 * @brief RADARPoint
 * @brief define custom point type containing radar data
 ********************************************************/
struct EIGEN_ALIGN16 RADARPoint
{
  PCL_ADD_POINT4D;
  float velocity;
  float snr;
  float rcs;
  float confidence;
  float velocity_interval;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace tam::types::perception

POINT_CLOUD_REGISTER_POINT_STRUCT(
  tam::types::perception::XYZITPoint,
  (float, x,
   x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp))
POINT_CLOUD_REGISTER_POINT_STRUCT(
  tam::types::perception::XYZITAEDPoint,
  (float, x,
   x)(float, y, y)(float, z, z)(double, timestamp, timestamp)(float, intensity, intensity)(
    float, azimuth, azimuth)(float, elevation, elevation)(float, distance, distance))
POINT_CLOUD_REGISTER_POINT_STRUCT(
  tam::types::perception::RADARPoint,
  (float, x, x)(float, y, y)(float, z, z)(float, velocity, velocity)(float, snr, snr)(
    float, rcs, rcs)(float, confidence, confidence)(float, velocity_interval, velocity_interval))
