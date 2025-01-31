//  Copyright 2023 Simon Sagmeister
#pragma once
#include <cmath>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "tum_types_cpp/data_per_wheel.hpp"
namespace tam::types
{
enum class ErrorLvl { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };
enum class EmergencyLevel {
  NO_EMERGENCY = 0,
  EMERGENCY_STOP = 1,
  SOFT_EMERGENCY = 2,
  HARD_EMERGENCY = 3,
};
}  // namespace tam::types
namespace tam::types::common
{
struct Header
{
  uint32_t seq;  // Sequence counter
  uint64_t time_stamp_ns;
  std::string frame_id;
};
template <typename T>
struct Vector3D
{
  Vector3D() = default;
  Vector3D(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
  explicit Vector3D(std::vector<T> vec_in) : x(vec_in.at(0)), y(vec_in.at(1)), z(vec_in.at(2)) {}
  T x, y, z;
  Vector3D operator+(const Vector3D & other) const
  {
    return Vector3D(x + other.x, y + other.y, z + other.z);
  }
  Vector3D operator-(const Vector3D & other) const
  {
    return Vector3D(x - other.x, y - other.y, z - other.z);
  }
  // left multiplication with a factor
  friend Vector3D operator*(const double & factor, const Vector3D & obj)
  {
    return Vector3D(factor * obj.x, factor * obj.y, factor * obj.z);
  }
  // right multiplication with a factor = left multiplication
  Vector3D operator*(const double & factor) const { return factor * (*this); }
};
template <typename T>
struct Vector2D
{
  Vector2D() = default;
  Vector2D(T x_, T y_) : x(x_), y(y_) {}
  explicit Vector2D(std::vector<T> vec_in) : x(vec_in.at(0)), y(vec_in.at(1)) {}
  T x, y;
  Vector2D operator+(const Vector2D & other) const { return Vector2D(x + other.x, y + other.y); }
  Vector2D operator-(const Vector2D & other) const { return Vector2D(x - other.x, y - other.y); }
  // left multiplication with a factor
  friend Vector2D operator*(const double & factor, const Vector2D & obj)
  {
    return Vector2D(factor * obj.x, factor * obj.y);
  }
  friend Vector2D<T> operator/(const Vector2D<T> & obj, const double factor)
  {
    return Vector2D(obj.x / factor, obj.y / factor);
  }
  // right multiplication with a factor = left multiplication
  Vector2D operator*(const double & factor) const { return factor * (*this); }
};
/* Euler Angles in rad. Axes of rotation:

    1. Yaw (around Z)
    2. Pitch (around the new Y axis)
    3. Roll (around the new new X axis)
*/
struct EulerYPR
{
  explicit EulerYPR(const double yaw_, const double pitch_, const double roll_)
  : yaw(yaw_), pitch(pitch_), roll(roll_)
  {
  }
  double yaw, pitch, roll;
};
struct FrenetPose
{
  explicit FrenetPose(const double s_, const double d_, const double dpsi_)
  : s(s_), d(d_), dpsi(dpsi_)
  {
  }
  explicit FrenetPose(Eigen::Vector3d vec) : s(vec(0)), d(vec(1)), dpsi(vec(2)) {}
  double s, d, dpsi;
};
}  // namespace tam::types::common
